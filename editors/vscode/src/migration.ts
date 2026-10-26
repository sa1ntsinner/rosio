// The Migration view: what moving to another ROS 2 release asks of this workspace.
import * as vscode from "vscode";
import * as path from "node:path";
import {
  MIGRATION_EMPTY,
  buildErrorTitle,
  buildErrorWhere,
  buildErrors,
  buildMessage,
  buildPackageDescription,
  buildPackageLabel,
  buildSection,
  changeTitle,
  changeDescription,
  migrationSections,
  planMessage,
  type BuildSection,
  type PackageGroup,
  type Section,
} from "./migrationModel";
import type { BuildError, BuildPackage, MigrationChange, MigrationPlan } from "./protocol";
import type { Server } from "./server";

type Row = BuildSection | BuildPackage | BuildError | Section | PackageGroup | MigrationChange;

export class MigrationView implements vscode.TreeDataProvider<Row>, vscode.Disposable {
  private readonly changed = new vscode.EventEmitter<void>();
  readonly onDidChangeTreeData = this.changed.event;
  private readonly subscription: vscode.Disposable;
  private view?: vscode.TreeView<Row>;

  constructor(private readonly server: Server) {
    this.subscription = server.onMigrationPlan(() => this.refresh());
  }

  attach(view: vscode.TreeView<Row>): void {
    this.view = view;
    this.refresh();
  }

  private refresh(): void {
    const plan = this.server.migrationPlan;
    if (this.view) {
      this.view.description = plan ? `${plan.from} to ${plan.to}` : undefined;
      this.view.message = buildMessage(this.server.migrationBuild) ?? planMessage(plan);
    }
    void vscode.commands.executeCommand("setContext", "rosio.migrationPlanned", Boolean(plan));
    void vscode.commands.executeCommand(
      "setContext",
      "rosio.migrationRewrites",
      Boolean(plan?.changes.some((c) => c.replacement !== undefined)),
    );
    this.changed.fire();
  }

  getChildren(row?: Row): Row[] {
    if (!row) {
      const build = buildSection(this.server.migrationBuild);
      return [...(build ? [build] : []), ...migrationSections(this.server.migrationPlan)];
    }
    if ("kind" in row && row.kind === "build") return row.packages;
    // One error is shown on its package's row.
    if ("errors" in row) return row.errors.length === 1 ? [] : buildErrors(row);
    if ("packages" in row) {
      // One package needs no package level.
      return row.packages.length === 1 ? row.packages[0].changes : row.packages;
    }
    if ("changes" in row) return row.changes;
    return [];
  }

  getTreeItem(row: Row): vscode.TreeItem {
    if ("kind" in row && row.kind === "build") {
      const open = row.packages.length
        ? vscode.TreeItemCollapsibleState.Expanded
        : vscode.TreeItemCollapsibleState.None;
      const item = new vscode.TreeItem(row.label, open);
      item.description = row.description;
      item.iconPath = row.ok
        ? new vscode.ThemeIcon("pass", new vscode.ThemeColor("testing.iconPassed"))
        : new vscode.ThemeIcon("error", new vscode.ThemeColor("problemsErrorIcon.foreground"));
      item.tooltip = `Built in ${this.server.migrationBuild?.image}. Plan, apply or clear again and this result is dropped.`;
      return item;
    }
    if ("errors" in row) return this.buildPackageItem(row);
    if ("matches" in row) return this.buildErrorItem(row);
    if ("packages" in row) {
      const open = row.kind === "hand"
        ? vscode.TreeItemCollapsibleState.Expanded
        : vscode.TreeItemCollapsibleState.Collapsed;
      const item = new vscode.TreeItem(row.label, open);
      item.description = String(row.count);
      item.contextValue = `migration.${row.kind}`;
      return item;
    }
    if ("changes" in row) {
      const item = new vscode.TreeItem(row.package, vscode.TreeItemCollapsibleState.Expanded);
      item.description = String(row.changes.length);
      item.iconPath = new vscode.ThemeIcon("package");
      return item;
    }
    return this.changeItem(row);
  }

  private buildPackageItem(row: BuildPackage): vscode.TreeItem {
    if (row.errors.length === 1) {
      const item = this.buildErrorItem(row.errors[0]);
      item.label = buildPackageLabel(row.name);
      item.description = buildErrorTitle(row.errors[0]);
      return item;
    }
    const open = row.errors.length > 1
      ? vscode.TreeItemCollapsibleState.Expanded
      : vscode.TreeItemCollapsibleState.None;
    const item = new vscode.TreeItem(buildPackageLabel(row.name), open);
    item.description = buildPackageDescription(row);
    item.iconPath = new vscode.ThemeIcon(row.status === "skipped" ? "circle-slash" : "package");
    if (row.status === "skipped") item.tooltip = "Not built: a package it depends on failed, or the build stopped first.";
    return item;
  }

  private buildErrorItem(error: BuildError): vscode.TreeItem {
    const listed = error.status === "listed";
    const item = new vscode.TreeItem(buildErrorTitle(error));
    item.description = buildErrorWhere(error);
    item.iconPath = listed
      ? new vscode.ThemeIcon("lightbulb")
      : new vscode.ThemeIcon("warning", new vscode.ThemeColor("problemsWarningIcon.foreground"));
    const tooltip = new vscode.MarkdownString(undefined, true);
    tooltip.appendCodeblock(error.message, "text");
    if (listed) {
      tooltip.appendMarkdown("The plan lists this line:\n\n");
      for (const match of error.matches) tooltip.appendMarkdown(`- ${match.say}\n`);
    } else {
      tooltip.appendMarkdown("No change in the plan covers this line. It needs a fix by hand.");
    }
    item.tooltip = tooltip;
    const root = this.server.migrationBuild?.root;
    if (error.file && error.line && root) {
      item.command = {
        command: "vscode.open",
        title: "Open",
        arguments: [
          vscode.Uri.file(path.join(root, error.file)),
          { selection: new vscode.Range(error.line - 1, 0, error.line - 1, 0) },
        ],
      };
    }
    return item;
  }

  private changeItem(change: MigrationChange): vscode.TreeItem {
    const automatic = change.replacement !== undefined;
    const item = new vscode.TreeItem(changeTitle(change));
    item.description = changeDescription(change);
    const tooltip = new vscode.MarkdownString(undefined, true);
    tooltip.appendMarkdown(`${change.say}\n\n`);
    if (automatic) {
      tooltip.appendMarkdown("Apply writes:\n");
      tooltip.appendCodeblock(change.replacement ?? "", "text");
    }
    const when = {
      removed: `Stops building in ${change.release}.`,
      behaviour: `Builds, but behaves differently from ${change.release}.`,
    }[change.effect] ?? `Warns from ${change.release}.`;
    const since = change.works_from ? ` The new form builds from ${change.works_from} on.` : "";
    tooltip.appendMarkdown(`${when}${since}\n\n`);
    tooltip.appendMarkdown(`\`${change.file}:${change.line}\` · [source](${change.source})`);
    item.tooltip = tooltip;
    item.contextValue = automatic ? "migration.change.automatic" : "migration.change.hand";
    item.command = {
      command: "rosio.openMigrationChange",
      title: "Open",
      arguments: [change, this.server.migrationPlan?.root],
    };
    return item;
  }

  dispose(): void {
    this.subscription.dispose();
    this.changed.dispose();
  }
}

export async function openMigrationChange(change: MigrationChange, root?: string): Promise<void> {
  if (!root) return;
  const document = await vscode.workspace.openTextDocument(vscode.Uri.file(path.join(root, change.file)));
  const line = change.line - 1;
  const character = Array.from(document.lineAt(line).text).slice(0, change.column - 1).join("").length;
  const start = new vscode.Position(line, character);
  const end = document.positionAt(document.offsetAt(start) + change.found.length);
  await vscode.window.showTextDocument(document, { selection: new vscode.Range(start, end) });
}

export async function applyMigration(server: Server): Promise<void> {
  const plan = server.migrationPlan;
  if (!plan) {
    void vscode.window.showInformationMessage(MIGRATION_EMPTY);
    return;
  }
  const result: MigrationPlan | undefined = await server.migrate("Apply", { from: plan.from, to: plan.to });
  if (result?.applied?.diff) {
    const rewrites = plan.changes.filter((c) => c.replacement !== undefined).length;
    const diff = result.applied.diff;
    void vscode.window
      .showInformationMessage(`Applied ${rewrites} exact rewrites. The diff is in ${diff}.`, "Open diff")
      .then((choice) => choice && vscode.window.showTextDocument(vscode.Uri.file(diff)));
  }
}

export async function verifyMigration(server: Server): Promise<void> {
  const plan = server.migrationPlan;
  if (!plan) {
    void vscode.window.showInformationMessage(MIGRATION_EMPTY);
    return;
  }
  await vscode.window.withProgress(
    { location: vscode.ProgressLocation.Notification, title: `Building the workspace in ${plan.to}. A first build pulls the image and takes minutes.` },
    () => server.verifyMigration({ from: plan.from, to: plan.to }),
  );
}
