// The trees in the rosio side bar: what is wrong, what the atlas could not read, and what
// the system is made of.
import * as vscode from "vscode";
import {
  SEVERITY_ORDER,
  type Finding,
  type FrameRow,
  type LaunchEntity,
  type LaunchFile,
  type LaunchProfile,
  type Location,
  type Severity,
  type Unresolved,
} from "./protocol";
import type { Server } from "./server";
import { describeId, labelOf, KIND_NAMES } from "./ids";
import {
  actionName,
  entityDescription,
  entityTitle,
  fileDescription,
  fileTitle,
  frameDescription,
  frameGroups,
  modelTitle,
  notStartedReason,
  type ModelGroup,
  profileDescription,
  profileTitle,
  started,
} from "./launchModel";

const SEVERITY_TITLE: Record<Severity, string> = { error: "Errors", warn: "Warnings", info: "Notes" };
const SEVERITY_ICON: Record<Severity, vscode.ThemeIcon> = {
  error: new vscode.ThemeIcon("error", new vscode.ThemeColor("problemsErrorIcon.foreground")),
  warn: new vscode.ThemeIcon("warning", new vscode.ThemeColor("problemsWarningIcon.foreground")),
  info: new vscode.ThemeIcon("info", new vscode.ThemeColor("problemsInfoIcon.foreground")),
};

type FindingNode = { severity: Severity; findings: Finding[] } | Finding;

export class FindingsView implements vscode.TreeDataProvider<FindingNode> {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private findings: Finding[] = [];

  constructor(private readonly server: Server) {}

  async refresh(): Promise<void> {
    this.findings = await this.server.findings();
    this.changed.fire();
  }

  getChildren(node?: FindingNode): FindingNode[] {
    if (!node) {
      return SEVERITY_ORDER.map((severity) => ({
        severity,
        findings: this.findings.filter((finding) => finding.severity === severity),
      })).filter((group) => group.findings.length > 0);
    }
    return "findings" in node ? node.findings : [];
  }

  getTreeItem(node: FindingNode): vscode.TreeItem {
    if ("findings" in node) {
      const item = new vscode.TreeItem(
        SEVERITY_TITLE[node.severity],
        node.severity === "info"
          ? vscode.TreeItemCollapsibleState.Collapsed
          : vscode.TreeItemCollapsibleState.Expanded,
      );
      item.description = String(node.findings.length);
      item.iconPath = SEVERITY_ICON[node.severity];
      return item;
    }
    const item = new vscode.TreeItem(node.title, vscode.TreeItemCollapsibleState.None);
    const first = node.locations[0];
    item.description = first ? `${decodeURIComponent(first.uri.slice(first.uri.lastIndexOf("/") + 1))}:${first.line + 1}` : node.rule;
    item.tooltip = new vscode.MarkdownString(`${node.say}\n\n**Fix:** ${node.fix}\n\n\`${node.rule}\``);
    item.contextValue = "finding";
    if (first) item.command = openAt(first);
    return item;
  }

  dispose(): void {
    this.changed.dispose();
  }
}

/** What each kind of blind spot means to someone reading their own workspace. */
const WANT_TITLE: Record<string, string> = {
  param_value: "Parameter values",
  topic_name: "Topic names",
  node_name: "Node names",
  frame_id: "Frame names",
  other: "Other",
};

type UnresolvedNode = { want: string; rows: Unresolved[] } | Unresolved;

export class UnresolvedView implements vscode.TreeDataProvider<UnresolvedNode> {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private rows: Unresolved[] = [];

  constructor(private readonly server: Server) {}

  async refresh(): Promise<void> {
    this.rows = await this.server.unresolved();
    this.changed.fire();
  }

  getChildren(node?: UnresolvedNode): UnresolvedNode[] {
    if (!node) {
      const wants = [...new Set(this.rows.map((row) => row.want))].sort(
        (a, b) => (a === "other" ? 1 : b === "other" ? -1 : a.localeCompare(b)),
      );
      return wants.map((want) => ({ want, rows: this.rows.filter((row) => row.want === want) }));
    }
    return "rows" in node ? node.rows : [];
  }

  getTreeItem(node: UnresolvedNode): vscode.TreeItem {
    if ("rows" in node) {
      const item = new vscode.TreeItem(
        WANT_TITLE[node.want] ?? node.want,
        vscode.TreeItemCollapsibleState.Collapsed,
      );
      item.description = String(node.rows.length);
      return item;
    }
    const item = new vscode.TreeItem(node.expr || node.reason, vscode.TreeItemCollapsibleState.None);
    item.description = node.owner;
    const fixes = node.resolvableBy?.length ? `\n\n**Resolvable by:** ${node.resolvableBy.join(", ")}` : "";
    item.tooltip = new vscode.MarkdownString(
      `\`${node.expr}\` in ${node.owner}\n\n**Why:** ${node.reason.replaceAll("_", " ")}${fixes}`,
    );
    const first = node.locations[0];
    if (first) item.command = openAt(first);
    return item;
  }

  dispose(): void {
    this.changed.dispose();
  }
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

type SystemNode = { kind: string; ids: string[] } | { id: string };

/** Kinds in the order an engineer reads a system: what runs, what it says, how it is set. */
const KIND_ORDER = ["profile", "node", "topic", "service", "action", "param", "frame", "launchfile", "pkg", "plugin", "bag"];

export class SystemView implements vscode.TreeDataProvider<SystemNode> {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private groups: Record<string, string[]> = {};

  constructor(private readonly server: Server) {}

  async refresh(): Promise<void> {
    this.groups = await this.server.children();
    this.changed.fire();
  }

  getChildren(node?: SystemNode): SystemNode[] {
    if (!node) {
      return KIND_ORDER.filter((kind) => this.groups[kind]?.length).map((kind) => ({
        kind,
        ids: this.groups[kind],
      }));
    }
    return "ids" in node ? node.ids.map((id) => ({ id })) : [];
  }

  getTreeItem(node: SystemNode): vscode.TreeItem {
    if ("ids" in node) {
      const item = new vscode.TreeItem(KIND_NAMES[node.kind] ?? node.kind, vscode.TreeItemCollapsibleState.Collapsed);
      item.description = String(node.ids.length);
      return item;
    }
    const item = new vscode.TreeItem(labelOf(node.id), vscode.TreeItemCollapsibleState.None);
    item.description = describeId(node.id);
    item.tooltip = node.id;
    item.command = { command: "rosio.showEntity", title: "Show in the atlas", arguments: [node.id] };
    if (/^(node|topic|service|action):/.test(node.id)) item.contextValue = "rosio.graphable";
    return item;
  }

  dispose(): void {
    this.changed.dispose();
  }
}

type LaunchNode = LaunchProfile | LaunchFile | LaunchEntity;

export class LaunchView implements vscode.TreeDataProvider<LaunchNode> {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private profiles: LaunchProfile[] = [];

  constructor(private readonly server: Server) {}

  async refresh(): Promise<void> {
    this.profiles = await this.server.launchTree();
    this.changed.fire();
  }

  getChildren(node?: LaunchNode): LaunchNode[] {
    if (!node) return this.profiles;
    if ("entries" in node) return node.entries;
    return node.children ?? [];
  }

  getTreeItem(node: LaunchNode): vscode.TreeItem {
    if ("entries" in node) {
      const item = new vscode.TreeItem(profileTitle(node), vscode.TreeItemCollapsibleState.Expanded);
      item.description = profileDescription(node);
      item.tooltip = new vscode.MarkdownString(
        [node.label, ...(node.command ?? "").split("\n").map((line) => `\`${line}\``)].join("\n\n"),
      );
      item.iconPath = new vscode.ThemeIcon("play");
      return item;
    }
    const children = node.children?.length ? vscode.TreeItemCollapsibleState.Expanded : vscode.TreeItemCollapsibleState.None;
    if ("arg_declarations" in node) {
      const item = new vscode.TreeItem(fileTitle(node), children);
      if (node.location) item.resourceUri = vscode.Uri.parse(node.location.uri);
      item.description = fileDescription(node);
      item.tooltip = node.label;
      if (node.location) item.command = openAt(node.location);
      return item;
    }
    const item = new vscode.TreeItem(entityTitle(node), children);
    item.description = entityDescription(node);
    const trail = node.id.split("|").slice(2).map((segment) => actionName(decodeURIComponent(segment))).join(" / ");
    const reason = notStartedReason(node);
    item.tooltip = reason ? `${trail}\n\n${reason}` : trail;
    if (node.kind !== "include") {
      item.iconPath = new vscode.ThemeIcon(started(node) ? LAUNCH_ICON[node.kind] : "circle-slash");
    }
    if (node.location) item.command = openAt(node.location);
    if (node.nodes.length) item.contextValue = "rosio.graphable";
    return item;
  }

  dispose(): void {
    this.changed.dispose();
  }
}

const LAUNCH_ICON: Record<Exclude<LaunchEntity["kind"], "include">, string> = {
  node: "server-process",
  composable: "server-process",
  container: "server-environment",
  group: "folder",
  other: "circle-outline",
};

type FrameNode = ModelGroup | { frame: FrameRow; siblings: FrameRow[] };

export class FramesView implements vscode.TreeDataProvider<FrameNode> {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private groups: ModelGroup[] = [];
  private findings = new Map<string, Finding>();

  constructor(private readonly server: Server) {}

  async refresh(): Promise<void> {
    const [roots, findings] = await Promise.all([this.server.frameTree(), this.server.findings()]);
    this.groups = frameGroups(roots);
    this.findings = new Map(findings.map((finding) => [finding.id, finding]));
    this.changed.fire();
  }

  getChildren(node?: FrameNode): FrameNode[] {
    if (!node && this.groups.length === 1) return frames(this.groups[0].roots);
    if (!node) return this.groups;
    return frames("roots" in node ? node.roots : node.frame.children);
  }

  getTreeItem(node: FrameNode): vscode.TreeItem {
    if ("roots" in node) {
      const item = new vscode.TreeItem(
        node.model ? modelTitle(node.model).title : "Not in a robot model",
        vscode.TreeItemCollapsibleState.Collapsed,
      );
      item.description = node.model ? modelTitle(node.model).pkg : "broadcast or looked up in code";
      item.iconPath = new vscode.ThemeIcon(node.model ? "file-code" : "code");
      return item;
    }
    return this.frameItem(node);
  }

  private frameItem({ frame, siblings }: { frame: FrameRow; siblings: FrameRow[] }): vscode.TreeItem {
    const item = new vscode.TreeItem(
      frame.label ?? frame.id,
      frame.children.length ? vscode.TreeItemCollapsibleState.Expanded : vscode.TreeItemCollapsibleState.None,
    );
    item.description = frameDescription(frame, siblings);
    const findings = frame.findings.map((id) => this.findings.get(id)).filter((f): f is Finding => Boolean(f));
    const worst = SEVERITY_ORDER.find((severity) => findings.some((f) => f.severity === severity));
    item.iconPath = worst ? SEVERITY_ICON[worst] : new vscode.ThemeIcon("circle-small");
    const lines = findings.map((f) => `**${f.title}**\n\n${f.say}`);
    if (frame.xyz) lines.push(`xyz ${frame.xyz.join(" ")}, rpy ${(frame.rpy ?? []).join(" ")}`);
    if (frame.model) item.contextValue = "rosio.inModel";
    if (frame.model) lines.push(`Model: ${frame.model.replace(/^file:/, "").replace("|", "/")}`);
    if (lines.length) item.tooltip = new vscode.MarkdownString(lines.join("\n\n"));
    const at = frame.location ?? frame.broadcasters[0]?.location ?? frame.listeners[0]?.location;
    if (at) item.command = openAt(at);
    return item;
  }

  dispose(): void {
    this.changed.dispose();
  }
}

function frames(rows: FrameRow[]): FrameNode[] {
  return rows.map((frame) => ({ frame, siblings: rows }));
}
