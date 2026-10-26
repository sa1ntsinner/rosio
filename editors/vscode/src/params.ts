// The Parameters view: every parameter a profile sets, the value that wins and the lines it beat.
import * as vscode from "vscode";
import {
  candidateDescription,
  formatValue,
  mechanismName,
  nodeDescription,
  notLaunched,
  paramDescription,
  paramIcon,
  paramPicks,
  where,
} from "./paramsModel";
import type { Location, ParamCandidate, ParamRow, ParamTree } from "./protocol";
import type { Server } from "./server";

type Profile = { profile: string; tree: ParamTree };
type NodeRow = ParamTree["nodes"][number];
type Row = Profile | NodeRow | ParamRow | { candidate: ParamCandidate; param: ParamRow };

export class ParamsView implements vscode.TreeDataProvider<Row>, vscode.Disposable {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private profiles: Profile[] = [];
  /** Small systems open with every node expanded; a large one would be a wall of rows. */
  private open = false;

  constructor(private readonly server: Server) {}

  async refresh(): Promise<void> {
    const names = (await this.server.children())["profile"] ?? [];
    const trees = await Promise.all(names.map((profile) => this.server.paramTree(profile)));
    this.profiles = names
      .map((profile, i) => ({ profile, tree: trees[i] }))
      .filter((row) => row.tree.nodes.length > 0);
    this.open = this.profiles.reduce((sum, row) => sum + row.tree.nodes.reduce((n, node) => n + node.params.length, 0), 0) <= 40;
    this.changed.fire();
  }

  trees(): ParamTree[] {
    return this.profiles.map((row) => row.tree);
  }

  getChildren(row?: Row): Row[] {
    if (!row) return this.profiles.length === 1 ? this.profiles[0].tree.nodes : this.profiles;
    if ("tree" in row) return row.tree.nodes;
    if ("params" in row) return row.params;
    if ("candidates" in row && row.candidates.length > 1) {
      return row.candidates.map((candidate) => ({ candidate, param: row }));
    }
    return [];
  }

  getTreeItem(row: Row): vscode.TreeItem {
    if ("tree" in row) {
      const item = new vscode.TreeItem(row.profile.replace(/^profile:/, ""), vscode.TreeItemCollapsibleState.Expanded);
      item.description = `${row.tree.nodes.length} nodes`;
      item.iconPath = new vscode.ThemeIcon("play");
      return item;
    }
    if ("params" in row) {
      const item = new vscode.TreeItem(
        row.label,
        this.open ? vscode.TreeItemCollapsibleState.Expanded : vscode.TreeItemCollapsibleState.Collapsed,
      );
      item.description = nodeDescription(row);
      item.iconPath = new vscode.ThemeIcon(notLaunched(row) ? "circle-slash" : "server-process");
      if (notLaunched(row)) item.tooltip = `No node called ${row.label} runs in this profile, so these values are never read.`;
      else item.contextValue = "rosio.graphable";
      return item;
    }
    if ("candidate" in row) {
      const { candidate } = row;
      const item = new vscode.TreeItem(formatValue(candidate.value, candidate.valueType) || "unknown");
      item.description = candidateDescription(candidate);
      item.iconPath = new vscode.ThemeIcon(candidate.wins ? "check" : "circle-small");
      item.tooltip = [mechanismName(candidate), candidate.source.file, candidate.source.locator].filter(Boolean).join("\n");
      if (candidate.source.at) item.command = openAt(candidate.source.at);
      return item;
    }
    const item = new vscode.TreeItem(
      row.name,
      row.candidates.length > 1 ? vscode.TreeItemCollapsibleState.Expanded : vscode.TreeItemCollapsibleState.None,
    );
    item.id = row.id;
    item.description = paramDescription(row);
    const icon = paramIcon(row);
    item.iconPath = icon === "warning"
      ? new vscode.ThemeIcon(icon, new vscode.ThemeColor("problemsWarningIcon.foreground"))
      : new vscode.ThemeIcon(icon);
    item.tooltip = paramTooltip(row);
    item.contextValue = "rosio.param";
    if (row.source?.at) item.command = openAt(row.source.at);
    return item;
  }

  dispose(): void {
    this.changed.dispose();
  }
}

function paramTooltip(row: ParamRow): vscode.MarkdownString {
  const text = new vscode.MarkdownString(`**${row.name}** ${code(paramDescription(row))}\n\n`);
  for (const candidate of row.candidates) {
    const mark = candidate.wins ? "wins" : "loses";
    text.appendMarkdown(`- ${code(formatValue(candidate.value, candidate.valueType))} ${mark}: ${mechanismName(candidate)}, ${where(candidate.source)}\n`);
  }
  if (row.findings) text.appendMarkdown(`\n${row.findings} finding${row.findings === 1 ? "" : "s"} about this parameter.`);
  return text;
}

/** A Markdown code span that holds any value, backticks included. */
export function code(value: string): string {
  const longest = Math.max(0, ...(value.match(/`+/g) ?? []).map((run) => run.length));
  const fence = "`".repeat(longest + 1);
  return `${fence} ${value} ${fence}`;
}

function openAt(location: Location): vscode.Command {
  return {
    command: "vscode.open",
    title: "Open",
    arguments: [
      vscode.Uri.parse(location.uri),
      { selection: new vscode.Range(location.line, 0, location.line, 0) } satisfies vscode.TextDocumentShowOptions,
    ],
  };
}

/** One box for every parameter of every profile: type a node or a name, land on the winning line. */
export async function findParameter(view: ParamsView): Promise<void> {
  const trees = view.trees();
  const many = trees.length > 1;
  const items = trees.flatMap((tree) =>
    paramPicks(tree).map((pick) => ({
      ...pick,
      detail: many ? `${pick.detail} · ${tree.profile?.replace(/^profile:/, "")}` : pick.detail,
    })),
  );
  if (!items.length) {
    void vscode.window.showInformationMessage("No parameters in the atlas yet. Extract it first.");
    return;
  }
  const chosen = await vscode.window.showQuickPick(items, {
    title: "Find parameter",
    placeHolder: "A parameter or a node, for example use_sim_time or /controller_server",
    matchOnDescription: true,
    matchOnDetail: true,
  });
  const at = chosen?.param.source?.at ?? chosen?.param.candidates[0]?.source.at;
  if (at) await vscode.commands.executeCommand(openAt(at).command, ...(openAt(at).arguments ?? []));
}
