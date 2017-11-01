/**
 * This snippet listens for serial methods and evals for callback
 * Bottom has a couple helpful functions for testing execution
 */
function actionComplete(title, data) {
  console.log('CALLBACK:', title, data);
}

var cmd = "";
Serial1.setup(9600);
Serial1.on('data', function (data) {
  cmd += data;
  var idx = cmd.indexOf("\r");
  while (idx >= 0) { 
    var line = cmd.substr(0, idx);
    cmd = cmd.substr(idx + 1);
    console.log(line);
    // try eval yikes :/
    try {
      eval(line);
    } catch(e) {
      console.log(e);
    }
    idx = cmd.indexOf("\r");
  }
});

// Demo example of talking to methods
function actionStart(str) {
  console.log('START: G');
  Serial1.println("G");
}