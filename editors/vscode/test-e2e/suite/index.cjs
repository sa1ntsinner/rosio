// The mocha entry VS Code loads inside the extension host.
const path = require("node:path");
const Mocha = require("mocha");

exports.run = function run() {
  const mocha = new Mocha({ ui: "bdd", timeout: 60000, color: true });
  mocha.addFile(path.join(__dirname, "extension.test.cjs"));
  return new Promise((resolve, reject) => {
    mocha.run((failures) => (failures ? reject(new Error(`${failures} tests failed`)) : resolve()));
  });
};
