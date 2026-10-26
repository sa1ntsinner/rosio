// rosio for VS Code: the language server, the side bar, the atlas panel, the system graph and the status bar.
import * as vscode from "vscode";
import { MigrationView, openMigrationChange, applyMigration, verifyMigration } from "./migration";
import { RELEASES, targetReleases } from "./migrationModel";
import { labelOf } from "./ids";
import type { Summary } from "./protocol";
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

  const refresh = async () => {
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
    if (!summary?.loaded) {
      status.text = "$(circle-outline) rosio";
      status.tooltip = "No atlas yet. Click to open the atlas panel.";
    } else {
      const { error, warn } = summary.findings;
      const profile = summary.profiles[0] ? `${summary.profiles[0].replace(/^profile:/, "")} · ` : "";
      const problems = error ? `$(error) ${error}` : warn ? `$(warning) ${warn}` : "$(check)";
      status.text = `rosio · ${profile}${problems}`;
      status.tooltip = new vscode.MarkdownString(
        [
          `**${error} errors, ${warn} warnings**`,
          `Layers: ${summary.layers.join(", ") || "none"}`,
          `Atlas: ${await atlasAge(summary.atlasPath)}`,
          "Click to open the atlas.",
        ].join("\n\n"),
      );
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
    vscode.window.registerTreeDataProvider("rosio.findings", findings),
    vscode.window.registerTreeDataProvider("rosio.unresolved", unresolved),
    vscode.window.registerTreeDataProvider("rosio.system", system),
    vscode.window.registerTreeDataProvider("rosio.launch", launch),
    vscode.window.registerTreeDataProvider("rosio.frames", frames),
    vscode.window.registerTreeDataProvider("rosio.params", params),
    vscode.commands.registerCommand("rosio.findParameter", () => findParameter(params)),
    server.onDidChange(() => void refresh()),
    vscode.commands.registerCommand("rosio.openAtlas", () => panel.showAtlas()),
    vscode.commands.registerCommand("rosio.showGraph", (focus?: unknown) => graph.show(graphFocus(focus))),
    vscode.commands.registerCommand("rosio.showRobot", () => robot.show()),
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
      detail: hit.layers.length ? hit.layers.join(" · ") : "known by name only",
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
