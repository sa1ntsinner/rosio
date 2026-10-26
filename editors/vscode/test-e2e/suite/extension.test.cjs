// The extension against a real language server and the fixture atlas.
const assert = require("node:assert");
const path = require("node:path");
const vscode = require("vscode");

const repository = vscode.workspace.workspaceFolders[0].uri.fsPath;
const refused = vscode.Uri.file(
  path.join(repository, "tests", "fixtures", "ws_static", "src", "fx_desc", "xacro", "fx_needs_arg.urdf.xacro"),
);

async function until(what, read, timeout = 30000) {
  const start = Date.now();
  for (;;) {
    const value = await read();
    if (value) return value;
    if (Date.now() - start > timeout) throw new Error(`timed out waiting for ${what}`);
    await new Promise((done) => setTimeout(done, 250));
  }
}

describe("rosio", () => {
  before(async () => {
    const extension = vscode.extensions.getExtension("sa1ntsinner.rosio");
    assert.ok(extension, "the extension is installed in the development host");
    const api = await extension.activate();
    let last;
    const summary = await until("the atlas", async () => {
      last = await api.summary();
      return last && last.loaded ? last : undefined;
    }).catch((error) => {
      console.log(`last summary: ${JSON.stringify(last)}, server error: ${api.serverError()}`);
      throw error;
    });
    console.log(`atlas loaded: ${summary.findings.warn} warn, layers ${summary.layers.join(", ")}`);
  });

  it("puts a finding on the file it cites", async () => {
    await vscode.window.showTextDocument(await vscode.workspace.openTextDocument(refused));
    const diagnostics = await until("diagnostics", () => {
      const found = vscode.languages.getDiagnostics(refused);
      return found.length ? found : undefined;
    });
    assert.strictEqual(diagnostics[0].source, "rosio");
    assert.strictEqual(String(diagnostics[0].code), "extract.input_refused");
    assert.strictEqual(diagnostics[0].severity, vscode.DiagnosticSeverity.Warning);
    const actions = await vscode.commands.executeCommand(
      "vscode.executeCodeActionProvider",
      refused,
      diagnostics[0].range,
    );
    assert.ok(
      actions.some((action) => action.title === "Waive this finding in rosio-waivers.yaml"),
      "a rosio finding offers a waiver",
    );
  });

  it("opens the atlas panel and finds a topic", async () => {
    const labels = () => vscode.window.tabGroups.all.flatMap((group) => group.tabs.map((tab) => tab.label));
    await vscode.commands.executeCommand("rosio.openAtlas");
    await until("the atlas panel", () => labels().includes("rosio atlas"));
    await vscode.commands.executeCommand("rosio.showEntity", "topic:/fx/cmd_vel_sim");
    await until("the entity card", () => labels().includes("/fx/cmd_vel_sim"));
  });

  it("opens the system graph, whole and around one topic", async () => {
    const labels = () => vscode.window.tabGroups.all.flatMap((group) => group.tabs.map((tab) => tab.label));
    await vscode.commands.executeCommand("rosio.showGraph");
    await until("the graph", () => labels().includes("System graph"));
    await vscode.commands.executeCommand("rosio.showGraph", "topic:/fx/cmd_vel_sim");
    await until("the focused graph", () => labels().includes("Graph: /fx/cmd_vel_sim"));
  });

  it("opens the robot model", async () => {
    const labels = () => vscode.window.tabGroups.all.flatMap((group) => group.tabs.map((tab) => tab.label));
    await vscode.commands.executeCommand("rosio.showRobot");
    await until("the robot model", () => labels().some((label) => label.startsWith("Robot: ")));
  });

  it("answers go to symbol with atlas entities", async () => {
    const symbols = await until("workspace symbols", async () => {
      const found = await vscode.commands.executeCommand("vscode.executeWorkspaceSymbolProvider", "base_link");
      return found && found.length ? found : undefined;
    });
    assert.ok(symbols.some((symbol) => symbol.name === "base_link"));
  });
  it("plans migration, offers an exact edit, drops stale diagnostics and clears", async () => {
    const uri = vscode.Uri.file(path.join(repository, "editors/vscode/test-e2e/fixture/upgrade/CMakeLists.txt"));
    const document = await vscode.workspace.openTextDocument(uri);
    await vscode.window.showTextDocument(document);
    await vscode.commands.executeCommand("rosio.planMigration", "lyrical", "humble");
    const diagnostics = await until("migration diagnostics", () => {
      const rows = vscode.languages.getDiagnostics(uri).filter(d => d.source === "rosio migrate");
      return rows.length === 2 ? rows : undefined;
    });
    const automatic = diagnostics.find(d => d.code.value === "fixture.automatic");
    assert.ok(automatic);
    assert.strictEqual(automatic.range.start.character, 4);
    assert.strictEqual(document.getText(automatic.range), "old");
    const actions = await vscode.commands.executeCommand("vscode.executeCodeActionProvider", uri, automatic.range, undefined, 10);
    const fix = actions.find(a => a.title === "Apply: new");
    assert.ok(fix && fix.edit, JSON.stringify(actions));
    await vscode.workspace.applyEdit(fix.edit);
    await until("stale migration removed", () => vscode.languages.getDiagnostics(uri).filter(d => d.source === "rosio migrate").length === 1);
    await vscode.commands.executeCommand("workbench.action.files.revert");
    await vscode.commands.executeCommand("rosio.applyMigration");
    await until("apply plans remaining manual changes", () => {
      const rows = vscode.languages.getDiagnostics(uri).filter(d => d.source === "rosio migrate");
      return rows.length === 1 && rows[0].code.value === "fixture.manual";
    });
    await vscode.commands.executeCommand("rosio.clearMigration");
    await until("migration cleared", () => !vscode.languages.getDiagnostics(uri).some(d => d.source === "rosio migrate"));
  });
});
