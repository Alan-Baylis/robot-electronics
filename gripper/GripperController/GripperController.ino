/*
GripperController -- firmware to control a robotic gripper with a Herkulex servo
(c) Mark Leavitt 2017
Additions: Trevorjtclarke
Version: 20170903
Hardware requirements:
  Teensy 3.x microcontroller
  Dongbu Robotics DRS-0101 servo
  Sparkful VL6180x range finder breakout
  Force sensitive resistor with opamp front-end on PCB GripperController GC001
License:
   This library is free software; you can redistribute it and/or
   modify it under the terms of the GNU Lesser General Public
   License as published by the Free Software Foundation; either
   version 2.1 of the License, or (at your option) any later version.

   This library is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
   Lesser General Public License for more details.

   You should have received a copy of the GNU Lesser General Public
   License along with this library; if not, write to the Free Software
   Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
// Libraries
  #include "HerkulexT.h"       // Note: ensure HerkulexT.h, HerkulexT.cpp are in same directory as this source file
  #include <Wire.h>
  #include <SparkFun_VL6180X.h>
  #include <EEPROM.h>
// Objects
  #define VL6180X_ADDRESS 0x29
  VL6180x rangeFinder(VL6180X_ADDRESS);
// Constants
  const int servoID = 0xFD;     // This is the default servo ID of 253
  const float openJawAngle = 45.0;
// Global variables
  bool haveRangeFinder = false;
  // Real time measurement values
  int dist;
  int pressure;
  int newPressure;
  float jawAngle;
  float newJawAngle;
  // Settings (can be saved in EEPROM)
  int16_t jawPositionClosed;
  int16_t pressureOffset;
  int16_t detectObjectRange;
  int16_t movementTime;
  int16_t learnedPressure;
  int16_t jawPositionLearned;
  int16_t holdTime;
  int16_t waitTime;
  int16_t streamRate;
  bool isDetecting = false;
  // State variables
  bool objectGrabbed = false;
  bool isDetectionTriggered = false;
  bool isHolding = false;
  bool isReleasing = false;
  // Timing variables (unsigned long to be compatible with millis() function)
  unsigned long timeReleased = 0;
  unsigned long timeClosed = 0;
  unsigned long lastStreamTime = 0;

void setup(){
  // Open serial port for commands and responses over USB
  Serial.begin(115200);
    Serial1.begin(9600);
  // Start i2c for communication with range finder, then confirm communications with it
  Wire.begin();
  delay(100);
    Wire.beginTransmission(VL6180X_ADDRESS);
    if (Wire.endTransmission() == 0) {
      haveRangeFinder = true;
    }
  // Initialize rangeFinder with default settings
  if (haveRangeFinder) {
    rangeFinder.VL6180xInit();
    delay(100);
    rangeFinder.VL6180xDefautSettings();
  }
  // Recall settings from EEPROM
  retrieveSettings();
    // Open Serial port 2 and initialize the Herkulex servo
  HerkulexT.beginSerial2(115200);
  delay(100);
  initializeServo();
}
void loop() {
  processSerialInput();
  processMachineInput();
  // If desired angle has changed, move the servo.
  if (newJawAngle != jawAngle) {
      HerkulexT.moveOne(servoID, calcPosition(newJawAngle), movementTime, H_LED_GREEN);
    jawAngle = newJawAngle;
  }
  // Read the contact pressure
  newPressure = readPressure();
  pressure = newPressure;
  // If there is a range finder, read the distance
  if (haveRangeFinder) {
    dist = rangeFinder.getDistance();
  }
  // If object has been grabbed, glow cyan
  if (objectGrabbed) {
    HerkulexT.setLed(servoID,H_LED_CYAN);
  }
  // Are we in the holding cycle?
  if (isHolding) {
    HerkulexT.setLed(servoID,H_LED_CYAN);
    if ((millis() - timeClosed) > (unsigned long)holdTime) {
      // Drop the object, begin release cycle
      releaseObject();
      isHolding = false;
      isReleasing = true;
      timeReleased = millis();
    }
  }
  // Are we in the releasing cycle?
  if (isReleasing) {
    HerkulexT.setLed(servoID,H_LED_OFF);
    if ((millis() - timeReleased) > (unsigned long)waitTime) {
      // Return to the autodetect state
      objectGrabbed = false;
      isHolding = false;
      isReleasing = false;
      isDetectionTriggered = false;
    }
  }
  // Is auto-detect active?
  if (isDetecting) {
    if (!isHolding && !isReleasing) {
      HerkulexT.setLed(servoID,H_LED_BLUE);
    }
    if (dist <= detectObjectRange) {
      // If triggered, grab the object
      if (!isDetectionTriggered) {
        msgSend("Object detected at " + String(dist) + " mm, grabbing object.", true);
        grabObject();
        isDetectionTriggered = true;
        isHolding = true;
        timeClosed = millis();
      }
    }
  }
  // If not grabbed, detecting, holding, or releasing, servo should glow green
  if (!objectGrabbed && !isHolding && !isReleasing && !isDetecting) {
    HerkulexT.setLed(servoID,H_LED_GREEN);
  }
  if (streamRate > 0) {
    // Is it time for a streamed report?
    if ((millis() - lastStreamTime) > (unsigned long)streamRate) {
      // Stream out a report
      msgSend("Angle: " + String(jawAngle), false);
      msgSend(", Distance (mm): " + String(dist), false);
      msgSend(", Pressure: " + String(newPressure), true);
      lastStreamTime = millis();
    }
  }
  // Check servo status, and recover if there is an error
  int status = HerkulexT.stat(servoID);
  if (status > 0) {
    msgSend("Servo error #" + String(status), true);
    msgSend("Automatically rebooting servo...", true);
    initializeServo();
  }
  delay(50);
}
void grabObject() {
  // If no learned position or pressure, just close the jaws to zero immediately
  if (learnedPressure == 0) {
    newJawAngle = 0.0;
  } else {
    newJawAngle = calcAngle(jawPositionLearned);
  }
  objectGrabbed = true;
}
void releaseObject() {
  jawAngle = 0;
  newJawAngle = openJawAngle;
  objectGrabbed = false;
}
void initializeServo() {
  // Perform the necessary tasks for a cold startup, or a recovery from error condition
  HerkulexT.reboot(servoID);
  delay(1000);
  HerkulexT.initialize();
  delay(1000);
  // On next loop, ensure servo moves to open jaw position
  jawAngle = 0.0;
  newJawAngle = openJawAngle;
}
void calibrateServo() {
  const int startCalPosition = 800;
  // Begin with jaws open
  bool calibrated = false;
  int jawPosition = startCalPosition;
  HerkulexT.moveOne(servoID, jawPosition, 100, H_LED_BLUE);
  delay(100);
  // Calibrate the pressure sensor
  calibratePressure();
  msgSend("FSR offset calibrated at: " + String(pressureOffset), true);
  // Now calibrate the closed jaw position
  int newPressure = 0;
  while (!calibrated) {
    // Move the servo using digital positioning, no offset. Glow blue during calibration.
    HerkulexT.moveOne(servoID, jawPosition, 50, H_LED_BLUE);
    // Check pressure and keep closing jaws by increments that depend on pressure detected
    newPressure = readPressure();
    if (newPressure < 10) {
      jawPosition = jawPosition + 1;
    } else {
      // Threshhold pressure reached. Save position, calculate angle.
      calibrated = true;
      jawPositionClosed = jawPosition;
      // Send message, and turn LED back to green
      jawAngle = 0;
      newJawAngle = openJawAngle;
      msgSend("Calibration completed! Closed jaw position is: " + String(jawPositionClosed), true);
      HerkulexT.setLed(servoID,H_LED_GREEN);
    }
    delay(50);
  }
}
void learnObject() {
  // Only learn if learnedPressure setting is non-zero
  if (learnedPressure == 0) {
    return;
  }
  // Begin by opening the jaws to the standard open position (should already be there though)
  int jawPosition = calcPosition(openJawAngle);
  HerkulexT.moveOne(servoID, jawPosition, 100, H_LED_BLUE);
  delay(100);
  // If we have a rangefinder, wait for object to come into range and stabilize
  if (haveRangeFinder) {
    bool comingIntoRange = true;
    int previousDistance = 255;
    // Keep measuring distance as long as it's decreasing.
    // Stop when it increases by 4 mm or more.
    while (comingIntoRange) {
      dist = rangeFinder.getDistance();
      // Determine if we are in range, and if the distance has not changed
      if ((dist < 100) && (abs(dist - previousDistance) < 4)) {
        // Use this as the distance
        comingIntoRange = false;
        detectObjectRange = dist;
        msgSend("Detected object at range: " + String(detectObjectRange) + ", please hold it steadily there.", true);
      } else {
        previousDistance = dist;
      }
      delay(100);
    }
    isDetecting = true;
    isDetectionTriggered = false;
  } else {
    detectObjectRange = 0;
    isDetecting = false;
    isDetectionTriggered = false;
  }
  // Now we close the jaws to determine the angle and pressure
  int newPressure = 0;
  bool learned = false;
  // Begin with coarse measurement, then change to fine measurement
  bool coarseMeasurement = true;
  while (!learned) {
    // Move the servo using raw mapping (i.e. no offset). Glow blue during learning.
    HerkulexT.moveOne(servoID, jawPosition, 50, H_LED_BLUE);
    // Check pressure
    newPressure = readPressure();
    if (coarseMeasurement) {
      // What to do during coarse measurement
      if (newPressure < 5) {
        // Nothing detected, so close jaws quickly
        jawPosition = jawPosition + 8;
      } else {
        // As soon as any pressure is detected, back off and continue in fine mode
        jawPosition = jawPosition - 8;
        coarseMeasurement = false;
      }
    } else {
      if (newPressure < learnedPressure) {
        jawPosition = jawPosition + 1;
      } else {
        // We've reached the object and the pressure.
        learned = true;
        jawPositionLearned = jawPosition;
        float angleLearned = calcAngle(jawPositionLearned);
        // Now open the jaws, send message, and turn LED back to green
        msgSend("Object learned at distance: " + String(detectObjectRange) + ", jaw angle: " + String(angleLearned) +  ", pressure: " + String(newPressure), true);
      }
    }
    // Check servo status, reboot and bail out from learning process if necessary
    int status = HerkulexT.stat(servoID);
    if (status > 0) {
      msgSend("Servo error #" + String(status), true);
      msgSend("Unable to learn object.", true);
      msgSend("Automatically rebooting servo...", true);
      initializeServo();
      break;
    }
    delay(50);
  }
  releaseObject();
}
void saveSettings() {
  // Save calibration and settings in EEPROM
  EEPROM.write(0, 0x01);              // Set EEPROM memory valid flag
  EEPROM.write(1, lowByte(jawPositionClosed));  // Store all numbers in little-endian format
  EEPROM.write(2, highByte(jawPositionClosed));
  EEPROM.write(3, lowByte(pressureOffset));
  EEPROM.write(4, highByte(pressureOffset));
  EEPROM.write(5, lowByte(detectObjectRange));
  EEPROM.write(6, highByte(detectObjectRange));
  EEPROM.write(7, lowByte(movementTime));
  EEPROM.write(8, highByte(movementTime));
  EEPROM.write(9, lowByte(learnedPressure));
  EEPROM.write(10, highByte(learnedPressure));
  EEPROM.write(11, lowByte(jawPositionLearned));
  EEPROM.write(12, highByte(jawPositionLearned));
  EEPROM.write(13, lowByte(holdTime));
  EEPROM.write(14, highByte(holdTime));
  EEPROM.write(15, lowByte(waitTime));
  EEPROM.write(16, highByte(waitTime));
  EEPROM.write(17, lowByte(streamRate));
  EEPROM.write(18, highByte(streamRate));
}
void retrieveSettings() {
  if (EEPROM.read(0) ==  0x01) {    // Only load if valid data has been stored in EEPROM
    jawPositionClosed = EEPROM.read(1) + 256 * EEPROM.read(2);
    pressureOffset = EEPROM.read(3) + 256 * EEPROM.read(4);
    detectObjectRange = EEPROM.read(5) + 256 * EEPROM.read(6);
    movementTime = EEPROM.read(7) + 256 * EEPROM.read(8);
    learnedPressure = EEPROM.read(9) + 256 * EEPROM.read(10);
    jawPositionLearned = EEPROM.read(11) + 256 * EEPROM.read(12);
    holdTime = EEPROM.read(13) + 256 * EEPROM.read(14);
    waitTime = EEPROM.read(15) + 256 * EEPROM.read(16);
    streamRate = EEPROM.read(17) + 256 * EEPROM.read(18);
  } else {
    // EEPROM has not been written yet. Use defaults:
    jawPositionClosed = 850;
    pressureOffset = 10;
    detectObjectRange = 0;
    movementTime = 300;
    learnedPressure = 0;
    jawPositionLearned = 0;
    holdTime = 3000;
    waitTime = 3000;
    streamRate = 0;
  }
  // Must recalculate angle offset from jaw position
  //angleOffsetClosed = calcAngle(jawPositionClosed);
  // Must set boolean variable isDetecting appropriately
  if (detectObjectRange > 0 ) {
    isDetecting = true;
  } else {
    isDetecting = false;
  }
}
float calcAngle(int position) {
  // Calculates the jaw angle from a digital position
  int displacement = jawPositionClosed - position;
  float angle = 0.1625 * (float)displacement;
  return angle;
}
int calcPosition(float angle) {
  // Multiply by bits per degree (taking 2:1 gear ratio into account)
  float displacement = 6.154 * angle;
  // Round the displacement (actually truncate) and subtract from the closed position
  int position = jawPositionClosed - (int)displacement;
  return position;
}
int readPressure() {
  // Reads FSR to determine pressure exerted by jaws.
  // Return is pressure, range 0 - 999. Practical range: 10 to 250.
  int offset = (int) pressureOffset;  // Convert pressureOffset to signed integer for calculations
  const int deadBandThreshold = 2;  // Get rid of nuisance 1 and 2 bit noise
  const int numReadings = 16  ;   // Average 16 readings to reduce noise
  int sumPressureReadings = 0;
  for (int i = 0; i < numReadings; i++) {
    sumPressureReadings += analogRead(0);
  }
  int netPressure = (sumPressureReadings/numReadings) - offset;
  int reportedPressure = 0;
  // Implement dead band below threshhold
  if (abs(netPressure) > deadBandThreshold) {
    reportedPressure = netPressure;
  }
  // Max out at 999
  if (reportedPressure > 999) {
    reportedPressure = 999;
  }
  return reportedPressure;
}

void calibratePressure() {
  // Read FSR to determine pressureOffset (i.e. value with no force present)
  // Return is pressure, range 0 - 999. Practical range: 10 to 250.
  const int numReadings = 16  ;   // Average 16 readings to reduce noise
  int sumPressureReadings = 0;
  for (int i = 0; i < numReadings; i++) {
    sumPressureReadings += analogRead(0);
  }
  pressureOffset = (sumPressureReadings/numReadings);
  return;
}

void processSerialInput() {
  // Checks for and processes any serial input
  static String inputString = "";
  static boolean inputStringComplete = false;
  while (Serial.available()) {
    // Read and add the new character
    char inChar = (char)Serial.read();
    inputString += inChar;
    // if the incoming character is a newline, then the string is complete
    if (inChar == '\n') {
      inputStringComplete = true;
      break;                          // Stop here even if more characters are in buffer
    }
  }
  //   while (Serial1.available()) {
  //       // Read and add the new character
  //       char inChar = (char)Serial1.read();
  //       inputString += inChar;
  //       // if the incoming character is a newline, then the string is complete
  //       if (inChar == '\n') {
  //           inputStringComplete = true;
  //           break;                          // Stop here even if more characters are in buffer
  //       }
  // }
  // If a complete message is available, parse it, then clear the buffer before returning
  if (inputStringComplete == true) {
    parseCommand(inputString);
    inputString = "";
    inputStringComplete = false;
  }
}
void parseCommand(String stringToParse) {
  // Called to parse and respond to a completed command received via the serial port
  // The first character determines the command; the remaining characters are the "payload"
  // For supported commands, please see the code for case '?'
  char readChar = stringToParse.charAt(0);
  switch (readChar) {
    case '?':
      msgSend("Commands and responses:", true);
      msgSend("!      Hi! Responds with hello message.", true);
      msgSend("?      Get list of commands.", true);
      msgSend("       Space clears the screen by sending 24 blank lines", true);
      msgSend("g      GRAB the object (using learned size and pressure if present).", true);
      msgSend("r      RELEASE the object.", true);
      msgSend("c      CALIBRATE the jaw positioning.", true);
      msgSend("lnnn   LEARN object size and LIMIT pressure to nnn (range 20 - 200). l0 erases learned value.", true);
      msgSend("snnn   STREAM data (angle, pressure, distance) every nnn mSec. s0 disables reporting.", true);
      msgSend("annn   ANGLE of jaw, set to nnn degrees. a0 = fully closed, a90 = fully open.", true);
      msgSend("tnnn   TIMING for movements set to nnnn mSec. Higher values = slower movement.", true);
      msgSend("dnnn   Auto- DETECT object at distance nnn (millimeters) and grab it. d0 to disable.", true);
      msgSend("hnnn   HOLD the auto-detected object for nnn mSec, then release. h0 for indefinitely long hold.", true);
      msgSend("wnnn   WAIT nnn mSec after release before restarting auto-detect. w0 for no wait.", true);
      msgSend("m      MEMORIZE the current calibration and settings data in EEPROM.", true);
      msgSend("b      Re-BOOT the servo", true);
      break;
    case '!':
      msgSend("Hello from the GRIPPER!", true);
      break;
    case ' ':
      for (int i = 0; i < 24; i++) {
        msgSend("", true);
      }
      break;
    case 'g':
      msgSend("Grabbing object", true);
      grabObject();
      break;
    case 'r':
      msgSend("Releasing object.", true);
      releaseObject();
      break;
    case 'c':
      msgSend("Calibrating. Do not place any object between jaws during this step.", true);
      calibrateServo();
      break;
    case 'm':
      msgSend("Memorizing current calibration and settings.", true);
      saveSettings();
      msgSend("Calibration position: " + String(jawPositionClosed) + " bits", true);
      msgSend("Pressure offset: " + String(pressureOffset) + " units", true);
      msgSend("Learned jaw angle: " + String(calcAngle(jawPositionLearned)) + " deg", true);
      msgSend("Learned pressure: " + String(learnedPressure) + " units", true);
      msgSend("Auto-detection range: " + String(detectObjectRange) + " mm", true);
      msgSend("Movement time: " + String(movementTime) + " mSec (range 100 - 2800)", true);
      msgSend("Hold time: " + String(holdTime) + " mSec", true);
      msgSend("Wait time: " + String(waitTime) + " mSec", true);
      msgSend("Streaming data every " + String(streamRate) + " mSec", true);
      break;
    case 'l':
      learnedPressure = getCommandPayload(stringToParse);
      if (learnedPressure == 0) {
        // Set angle and distance to zero as well
        jawPositionLearned = jawPositionClosed;
        detectObjectRange = 0;
        msgSend("Learned position and pressure canceled.", true);
      } else {
        msgSend("Place object to be learned between jaws now.", true);
        msgSend("Learning in progress...", true);
      learnObject();
      }
      break;
    case 's':
      streamRate = getCommandPayload(stringToParse);
      if (streamRate > 0) {
        msgSend("Streaming data every " + String(streamRate) + " mSec", true);
      } else {
        msgSend("Streaming turned off.", true);
      }
      break;
    case 'a':
      newJawAngle = getCommandPayload(stringToParse);
      msgSend("Moving angle to: " + String(newJawAngle) + " deg", true);
      break;
    case 't':
      movementTime = constrain(getCommandPayload(stringToParse), 100, 2800);
      msgSend("Setting movement time to: " + String(movementTime) + " mSec", true);
      break;
    case 'd':
      detectObjectRange = getCommandPayload(stringToParse);
      if (detectObjectRange > 0) {
        isDetecting = true;
        isDetectionTriggered = false;
        msgSend("Auto-detecting objects at range: " + String(detectObjectRange) + " mm", true);
      } else {
        isDetecting = false;
        isDetectionTriggered = false;
        msgSend("Auto-detection turned off.", true);
      }
      break;
    case 'h':
      holdTime = getCommandPayload(stringToParse);
      msgSend("Hold time set to: " + String(holdTime) + " mSec", true);
      break;
    case 'w':
      waitTime = getCommandPayload(stringToParse);
      msgSend("Wait time set to: " + String(waitTime) + " mSec", true);
      break;
    case 'b':
      msgSend("Rebooting servo...", true);
      initializeServo();
      msgSend("Rebooted and initialized, jaws opening.", true);
      break;
    default:
      msgSend(readChar, false);
      msgSend(" is not a recognized command", true);
      break;
  }
}

// START TC ADDITIONS
// - Enables external serial to issue commands and receive Javascript responses
void processMachineInput() {
  // Checks for and processes any serial input
  static String inputString = "";
  static boolean inputStringComplete = false;
  while (Serial1.available()) {
    // Read and add the new character
    char inChar = (char)Serial1.read();
    inputString += inChar;
    // if the incoming character is a newline, then the string is complete
    if (inChar == '\n') {
      inputStringComplete = true;
      break;                          // Stop here even if more characters are in buffer
    }
  }
  // If a complete message is available, parse it, then clear the buffer before returning
  if (inputStringComplete == true) {
    parseMachineCommand(inputString);
    inputString = "";
    inputStringComplete = false;
  }
}
void parseMachineCommand(String stringToParse) {
  // Called to parse and respond to a completed command received via the serial port
  // The first character determines the command; the remaining characters are the "payload"
  // For supported commands, please see the code for case '?'
  char readChar = stringToParse.charAt(0);
  switch (readChar) {
    case '?':
      machineSend("actionComplete('Command Reference In Github')", true);
      break;
    case '!':
      machineSend("actionComplete('Hello from the GRIPPER!')", true);
      break;
    case 'g':
      machineSend("actionComplete('Grabbing')", true);
      grabObject();
      break;
    case 'r':
      machineSend("actionComplete('Releasing')", true);
      releaseObject();
      break;
    case 'c':
      machineSend("actionComplete('Calibrating')", true);
      calibrateServo();
      break;
    case 'm':
      // Special data return object
      machineSend("actionComplete('Memorizing', {", false);
        machineSend("jawPositionClosed\x3A" + String(jawPositionClosed) + ",", false);
        machineSend("pressureOffset\x3A" + String(pressureOffset) + ",", false);
        machineSend("jawPositionLearned\x3A" + String(calcAngle(jawPositionLearned)) + ",", false);
        machineSend("learnedPressure\x3A" + String(learnedPressure) + ",", false);
        machineSend("detectObjectRange\x3A" + String(detectObjectRange) + ",", false);
        machineSend("movementTime\x3A" + String(movementTime) + ",", false);
        machineSend("holdTime\x3A" + String(holdTime) + ",", false);
        machineSend("waitTime\x3A" + String(waitTime) + ",", false);
        machineSend("streamRate\x3A" + String(streamRate) + ",", false);
      machineSend("})", true);
      saveSettings();
      break;
    case 'l':
      learnedPressure = getCommandPayload(stringToParse);
      if (learnedPressure == 0) {
        // Set angle and distance to zero as well
        jawPositionLearned = jawPositionClosed;
        detectObjectRange = 0;
        machineSend("actionComplete('Learned position', {learnedPressure\x3A" + String(learnedPressure) + "})", true);
      } else {
        machineSend("actionComplete('Learning in progress')", true);
        learnObject();
      }
      break;
    case 's':
      streamRate = getCommandPayload(stringToParse);
      if (streamRate > 0) {
        machineSend("actionComplete('Streaming Data', {stream\x3A" + String(streamRate) + "})", true);
      } else {
        machineSend("actionComplete('Streaming Off')", true);
      }
      break;
    case 'a':
      newJawAngle = getCommandPayload(stringToParse);
      machineSend("actionComplete('Jaw Angle', {newJawAngle\x3A" + String(newJawAngle) + "})", true);
      break;
    case 't':
      movementTime = constrain(getCommandPayload(stringToParse), 100, 2800);
      machineSend("actionComplete('Movement Time', {movementTime\x3A" + String(movementTime) + "})", true);
      break;
    case 'd':
      detectObjectRange = getCommandPayload(stringToParse);
      if (detectObjectRange > 0) {
        isDetecting = true;
        isDetectionTriggered = false;
        machineSend("actionComplete('Auto-Range', {rangeOffset\x3A" + String(detectObjectRange) + "})", true);
      } else {
        isDetecting = false;
        isDetectionTriggered = false;
        machineSend("actionComplete('Auto-Range Off')", true);
      }
      break;
    case 'h':
      holdTime = getCommandPayload(stringToParse);
      machineSend("actionComplete('Hold Time', {holdTime\x3A" + String(holdTime) + "})", true);
      break;
    case 'w':
      waitTime = getCommandPayload(stringToParse);
      machineSend("actionComplete('Wait Time', {waitTime\x3A" + String(waitTime) + "})", true);
      break;
    case 'b':
      initializeServo();
      machineSend("actionComplete('Rebooted')", true);
      break;
    default:
      machineSend("actionComplete(" + String(readChar) + "' not recognized')", true);
      break;
  }
}
// END TC ADDITIONS

float getCommandPayload(String stringWithPayload) {
  // Extracts the numeric payload from a command
  // Just skip the first character to get the payload, and parse it as a float
  String payload = stringWithPayload.substring(1);
  return payload.toFloat();
}
void msgSend(String s, bool isComplete) {
  static String outputString = "";
  outputString += s;                                               // Add current string to the output buffer
  if (isComplete) {
    Serial.println(outputString);                   // Print it out
    outputString = "";                                               // Empty the buffer
  }
}
void machineSend(String s, bool isComplete) {
  static String outputString = "";
  outputString += s;
  if (isComplete) {
    Serial1.println(outputString);
    outputString = "";
  }
}
