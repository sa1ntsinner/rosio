// rosio for VS Code: the language server, the side bar, the atlas panel, the system graph and the status bar.
import * as vscode from "vscode";
import { MigrationView, openMigrationChange, applyMigration, verifyMigration } from "./migration";
import { RELEASES, targetReleases } from "./migrationModel";
import { labelOf } from "./ids";
import { layerName, layerWords, type Finding, type Summary } from "./protocol";
import { GraphPanel } from "./graph";
import { RobotPanel } from "./robot";
import { AtlasPanel } from "./panel";
import { ParamsView, findParameter } from "./params";
import { Server } from "./server";
import { FindingsView, FramesView, LaunchView, SystemView, UnresolvedView } from "./views";

/** What other extensions, and the end to end tests, can ask the running extension. */
export interface RosioApi {
  summary(): Promise<Summary | undefined>;
  /** Why the language server is not running, or undefined when it is. */
  serverError(): string | undefined;
}

export async function activate(context: vscode.ExtensionContext): Promise<RosioApi> {
  const output = vscode.window.createOutputChannel("rosio");
  const server = new Server(output, context.extensionPath);
  const findings = new FindingsView(server);
  const findingsTree = vscode.window.createTreeView("rosio.findings", { treeDataProvider: findings });
  const system = new SystemView(server);
  const unresolved = new UnresolvedView(server);
  const launch = new LaunchView(server);
  const frames = new FramesView(server);
  const params = new ParamsView(server);
  const migration = new MigrationView(server);
  const migrationTree = vscode.window.createTreeView("rosio.migration", { treeDataProvider: migration });
  migration.attach(migrationTree);
  const panel = new AtlasPanel(context.extensionUri, server);
  const graph = new GraphPanel(context.extensionUri, server);
  const robot = new RobotPanel(context.extensionUri, server);
  const status = vscode.window.createStatusBarItem(vscode.StatusBarAlignment.Left, 50);
  status.command = "rosio.openAtlas";
  status.name = "rosio";

  /** Files whose text the atlas is read from. */
  const READ = /\.(py|cpp|cc|hpp|h|ya?ml|xml|urdf|xacro|sdf|msg|srv|action|txt)$/;
  // After a save the findings on screen describe the file as it was. Extraction can take
  // seconds, so it stays a decision: the status bar says so and one click runs it.
  const markStale = (document: vscode.TextDocument) => {
    const config = vscode.workspace.getConfiguration("rosio");
    if (config.get<boolean>("extractOnSave", false) || !READ.test(document.fileName)) return;
    if (document.fileName.includes("rosio-out") || status.command === "rosio.extract") return;
    if (!status.text.startsWith("rosio ·")) return;
    status.text += " $(history)";
    status.tooltip = "A file was saved after this atlas was extracted. Click to extract again.";
    status.command = "rosio.extract";
  };

  const refresh = async () => {
    status.command = "rosio.openAtlas";
    await Promise.all([
      findings.refresh(),
      unresolved.refresh(),
      system.refresh(),
      launch.refresh(),
      frames.refresh(),
      params.refresh(),
      panel.update(),
      graph.update(),
      robot.update(true),
    ]);
    const summary = await server.summary();
    const fixable = (await server.findings()).filter((finding) => finding.fixable).length;
    void vscode.commands.executeCommand("setContext", "rosio.fixable", fixable > 0);
    const urgent = summary?.loaded ? summary.findings.error + summary.findings.warn : 0;
    findingsTree.badge = urgent ? { value: urgent, tooltip: `${urgent} errors and warnings` } : undefined;
    if (!summary?.loaded) {
      status.text = "$(circle-outline) rosio";
      status.tooltip = "No atlas yet. Click to open the atlas panel.";
    } else {
      const { error, warn } = summary.findings;
      const profile = summary.profiles[0] ? `${summary.profiles[0].replace(/^profile:/, "")} · ` : "";
      const problems = error ? `$(error) ${error}` : warn ? `$(warning) ${warn}` : "$(check)";
      status.text = `rosio · ${profile}${problems}`;
      const tooltip = new vscode.MarkdownString(
        [
          `**${error} errors, ${warn} warnings**`,
          `Read from: ${summary.layers.map(layerName).join(", ") || "nothing yet"}`,
          `Atlas: ${await atlasAge(summary.atlasPath)}`,
          [fixable ? `[Apply ${fixable} ${fixable === 1 ? "fix" : "fixes"}](command:rosio.fixAll)` : "", "[Extract again](command:rosio.extract)", "[Find](command:rosio.find)"]
            .filter(Boolean)
            .join(" · "),
        ].join("\n\n"),
      );
      tooltip.isTrusted = { enabledCommands: ["rosio.fixAll", "rosio.extract", "rosio.find"] };
      status.tooltip = tooltip;
    }
    status.show();
  };

  context.subscriptions.push(
    migration,
    migrationTree,
    vscode.commands.registerCommand("rosio.openMigrationChange", openMigrationChange),
    vscode.commands.registerCommand("rosio.planMigration", async (target?: string, start?: string) => {
      const from = start ?? server.migrationPlan?.from ?? process.env.ROS_DISTRO
        ?? await vscode.window.showQuickPick(RELEASES.slice(0, -1), {
          title: "Plan migration", placeHolder: "The ROS 2 release this workspace builds against now",
        });
      if (!from) return;
      const to = target ?? await vscode.window.showQuickPick(targetReleases(from), {
        title: `Plan migration from ${from}`, placeHolder: "Target ROS 2 release",
      });
      if (to) await server.migrate("Plan", { from, to });
    }),
    vscode.commands.registerCommand("rosio.applyMigration", () => applyMigration(server)),
    vscode.commands.registerCommand("rosio.verifyMigration", () => verifyMigration(server)),
    vscode.commands.registerCommand("rosio.clearMigration", () => server.migrate("Clear")),
    output,
    server,
    findings,
    unresolved,
    system,
    launch,
    frames,
    params,
    panel,
    graph,
    robot,
    status,
    findingsTree,
    vscode.window.registerTreeDataProvider("rosio.unresolved", unresolved),
    vscode.window.registerTreeDataProvider("rosio.system", system),
    vscode.window.registerTreeDataProvider("rosio.launch", launch),
    vscode.window.registerTreeDataProvider("rosio.frames", frames),
    vscode.window.registerTreeDataProvider("rosio.params", params),
    vscode.commands.registerCommand("rosio.findParameter", () => findParameter(params)),
    vscode.commands.registerCommand("rosio.fixFinding", (finding: Finding) => fixFinding(finding)),
    vscode.commands.registerCommand("rosio.fixAll", () => fixAll(server)),
    vscode.commands.registerCommand("rosio.copyFinding", (finding: Finding) => vscode.env.clipboard.writeText(findingText(finding))),
    vscode.commands.registerCommand("rosio.copyFindings", async () =>
      vscode.env.clipboard.writeText((await server.findings()).map(findingText).join("\n\n")),
    ),
    server.onDidChange(() => void refresh()),
    vscode.workspace.onDidSaveTextDocument(markStale),
    vscode.commands.registerCommand("rosio.openAtlas", () => panel.showAtlas()),
    vscode.commands.registerCommand("rosio.showGraph", (focus?: unknown) => graph.show(graphFocus(focus))),
    vscode.commands.registerCommand("rosio.showRobot", (from?: unknown) => robot.show(robotFocus(from))),
    vscode.commands.registerCommand("rosio.showEntity", (id: string) => panel.showEntity(id)),
    vscode.commands.registerCommand("rosio.extract", () => extract(server)),
    vscode.commands.registerCommand("rosio.find", () => find(server, panel)),
    vscode.commands.registerCommand("rosio.restartServer", () => server.restart()),
    vscode.commands.registerCommand("rosio.checkStamps", (uri?: vscode.Uri) => checkStamps(uri)),
  );

  await server.start();
  await refresh();
  return { summary: () => server.summary(), serverError: () => server.startError };
}

/** Applies the replacement the language server offers for a finding at the line it cites. */
async function applyFix(finding: Finding): Promise<vscode.TextDocument | undefined> {
  const at = finding.locations[0];
  if (!at) return undefined;
  const uri = vscode.Uri.parse(at.uri);
  const document = await vscode.workspace.openTextDocument(uri);
  const line = document.lineAt(at.line).range;
  const actions = await vscode.commands.executeCommand<vscode.CodeAction[]>("vscode.executeCodeActionProvider", uri, line);
  const fix = actions?.find((action) => action.isPreferred && action.edit);
  return fix?.edit && (await vscode.workspace.applyEdit(fix.edit)) ? document : undefined;
}

const LINE_MOVED = "The cited line changed since the atlas was built, so rosio did not edit it.";

async function fixFinding(finding: Finding): Promise<void> {
  const document = await applyFix(finding);
  if (!document) {
    void vscode.window.showWarningMessage(LINE_MOVED);
    return;
  }
  const line = finding.locations[0].line;
  await vscode.window.showTextDocument(document, { selection: new vscode.Range(line, 0, line, 0) });
}

/** Every exact fix in one step: edit, save the files it touched, extract again. */
async function fixAll(server: Server): Promise<void> {
  const fixable = (await server.findings()).filter((finding) => finding.fixable);
  const edited = new Set<vscode.TextDocument>();
  for (const finding of fixable) {
    const document = await applyFix(finding);
    if (document) edited.add(document);
    else void vscode.window.showWarningMessage(`${finding.title}: ${LINE_MOVED}`);
  }
  if (!edited.size) return;
  await Promise.all([...edited].map((document) => document.save()));
  await extract(server);
}

/** A finding as text a person or an assistant can act on without the editor. */
function findingText(finding: Finding): string {
  const at = finding.locations[0];
  const where = at ? `${vscode.workspace.asRelativePath(vscode.Uri.parse(at.uri))}:${at.line + 1}` : "";
  return [`${finding.severity}: ${finding.title} (${finding.rule})`, finding.say, `Fix: ${finding.fix}`, where].filter(Boolean).map(layerWords).join("\n");
}

/** How old the atlas on disk is, the way a person says it. */
async function atlasAge(atlasPath: string): Promise<string> {
  try {
    const stat = await vscode.workspace.fs.stat(vscode.Uri.file(atlasPath));
    const minutes = Math.round((Date.now() - stat.mtime) / 60000);
    if (minutes < 1) return "extracted just now";
    if (minutes < 60) return `extracted ${minutes} min ago`;
    const hours = Math.round(minutes / 60);
    return hours < 48 ? `extracted ${hours} h ago` : `extracted ${Math.round(hours / 24)} days ago`;
  } catch {
    return "not found";
  }
}

/** A frame row of the Frames view that a model places: show that model with the frame selected. */
function robotFocus(from: unknown): { frame: string; model: string } | undefined {
  const row = (from as { frame?: { id?: unknown; model?: unknown } } | undefined)?.frame;
  return typeof row?.id === "string" && typeof row.model === "string" ? { frame: row.id, model: row.model } : undefined;
}

/** A graph command reaches here from the palette, from code, or from a row of a side bar view. */
function graphFocus(from: unknown): string | null {
  if (typeof from === "string") return from;
  if (!from || typeof from !== "object") return null;
  const row = from as { id?: unknown; nodes?: { id: string }[] };
  if (row.nodes?.length) return row.nodes[0].id;
  return typeof row.id === "string" && /^(node|topic|service|action):/.test(row.id) ? row.id : null;
}

async function extract(server: Server): Promise<void> {
  await vscode.window.withProgress(
    { location: vscode.ProgressLocation.Window, title: "rosio: extracting the atlas" },
    () => server.extract(),
  );
}

/** One box for every object in the system, the way Go to Symbol works for code. */
async function find(server: Server, panel: AtlasPanel): Promise<void> {
  const pick = vscode.window.createQuickPick<vscode.QuickPickItem & { id: string }>();
  pick.placeholder = "A node, topic, parameter, frame or package";
  pick.matchOnDescription = true;
  let generation = 0;
  pick.onDidChangeValue(async (query) => {
    const mine = ++generation;
    if (!query.trim()) {
      pick.items = [];
      return;
    }
    pick.busy = true;
    const hits = await server.search(query.trim());
    if (mine !== generation) return;
    pick.items = hits.map((hit) => ({
      id: hit.id,
      label: labelOf(hit.id),
      description: hit.kind,
      detail: hit.layers.length ? hit.layers.map(layerName).join(" · ") : "known by name only",
    }));
    pick.busy = false;
  });
  pick.onDidAccept(() => {
    const chosen = pick.selectedItems[0];
    pick.hide();
    if (chosen) void panel.showEntity(chosen.id);
  });
  pick.onDidHide(() => pick.dispose());
  pick.show();
}

async function checkStamps(uri?: vscode.Uri): Promise<void> {
  const target =
    uri ??
    (
      await vscode.window.showOpenDialog({
        canSelectFolders: true,
        canSelectFiles: true,
        openLabel: "Check stamps",
        title: "A rosbag2 directory, an mcap file or a ROS 1 bag",
      })
    )?.[0];
  if (!target) return;
  const bag = target.fsPath.endsWith("metadata.yaml") ? vscode.Uri.joinPath(target, "..").fsPath : target.fsPath;
  const command = vscode.workspace.getConfiguration("rosio").get<string[]>("command", ["rosio"]);
  const terminal = vscode.window.createTerminal({ name: "rosio stamps" });
  terminal.show();
  terminal.sendText([...command, "stamps", bag, "--stats"].map(quote).join(" "));
}

function quote(argument: string): string {
  return /^[\w./:=@-]+$/.test(argument) ? argument : `'${argument.replace(/'/g, "'\\''")}'`;
}

export function deactivate(): void {}
