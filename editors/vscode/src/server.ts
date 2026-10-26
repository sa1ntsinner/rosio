// The language client for rosio-lsp, and the one place the extension talks to it.
import * as vscode from "vscode";
import { LanguageClient, type LanguageClientOptions, type ServerOptions } from "vscode-languageclient/node";
import { ATLAS_CHANGED, MIGRATION_PLAN, type MigrationBuild, type MigrationPlan, type AtlasChanged } from "./protocol";
import type { Entity, Finding, FrameRow, GraphData, LaunchProfile, MeshData, ParamTree, RobotModel, SearchHit, Summary, Unresolved } from "./protocol";
import { serverCommand } from "./serverPath";

export class Server implements vscode.Disposable {
  private client: LanguageClient | undefined;
  /** Why the server is not running, when it failed to start. */
  startError: string | undefined;
  private readonly changed = new vscode.EventEmitter<void>();
  /** Fires after the server published a new atlas, so views can refresh. */
  readonly onDidChange = this.changed.event;

  migrationPlan: MigrationPlan | undefined;
  private readonly migrationChanged = new vscode.EventEmitter<void>();
  readonly onMigrationPlan = this.migrationChanged.event;

  /** The last build of the plan in its target release; any new plan makes it stale and drops it. */
  migrationBuild: MigrationBuild | undefined;

  async migrate(verb: "Plan" | "Apply" | "Clear", args = {}): Promise<MigrationPlan | undefined> {
    return this.request<MigrationPlan>("workspace/executeCommand", { command: `rosio-lsp.migrate${verb}`, arguments: [args] });
  }

  async verifyMigration(args: { from: string; to: string }): Promise<MigrationBuild | undefined> {
    const build = await this.request<MigrationBuild>("workspace/executeCommand", { command: "rosio-lsp.migrateVerify", arguments: [args] });
    if (build) {
      this.migrationBuild = build;
      this.migrationChanged.fire();
    }
    return build;
  }

  constructor(
    private readonly output: vscode.OutputChannel,
    private readonly extensionPath = "",
  ) {}

  async start(): Promise<void> {
    const config = vscode.workspace.getConfiguration("rosio");
    const command = config.get<string[]>("command", ["rosio"]);
    const serverOptions: ServerOptions = {
      command: serverCommand(config.get<string>("server.path"), this.extensionPath),
      args: [],
    };
    const clientOptions: LanguageClientOptions = {
      documentSelector: [
        { scheme: "file", pattern: "**/{CMakeLists.txt,*.cmake,package.xml}" },
        { scheme: "file", language: "cpp" },
        { scheme: "file", language: "c" },
        { scheme: "file", language: "python" },
        { scheme: "file", language: "yaml" },
        { scheme: "file", language: "xml" },
        { scheme: "file", pattern: "**/*.{urdf,xacro,launch,msg,srv,action}" },
      ],
      outputChannel: this.output,
      middleware: {
        handleDiagnostics: (uri, diagnostics, next) => {
          next(uri, diagnostics);
          this.scheduleChange();
        },
      },
      initializationOptions: {
        atlasPath: config.get<string>("atlasPath", "rosio-out/atlas.json"),
        extractCommand: [...command, ...config.get<string[]>("extractArgs", ["extract", "."])],
        extractOnSave: config.get<boolean>("extractOnSave", false),
        codeLens: config.get<boolean>("codeLens", true),
      },
    };
    this.client = new LanguageClient("rosio", "rosio", serverOptions, clientOptions);
    this.migrationPlan = undefined;
    this.migrationBuild = undefined;
    this.migrationChanged.fire();
    this.client.onNotification(MIGRATION_PLAN, (plan: MigrationPlan | null) => {
      this.migrationPlan = plan ?? undefined;
      this.migrationBuild = undefined;
      this.migrationChanged.fire();
    });
    this.client.onNotification(ATLAS_CHANGED, (_change: AtlasChanged) => this.scheduleChange());
    try {
      this.startError = undefined;
      await this.client.start();
      this.changed.fire();
    } catch (error) {
      this.startError = String(error);
      this.output.appendLine(`The rosio language server did not start: ${String(error)}`);
      // Not awaited: activation must not wait on somebody reading a message.
      void vscode.window
        .showErrorMessage(`The rosio language server did not start: ${String(error)}`, "Open setting")
        .then((pick) => {
          if (pick === "Open setting") {
            void vscode.commands.executeCommand("workbench.action.openSettings", "rosio.server.path");
          }
        });
    }
  }

  private pending: NodeJS.Timeout | undefined;

  /** One refresh per burst: a reload publishes diagnostics for every cited file at once. */
  private scheduleChange(): void {
    if (this.pending) clearTimeout(this.pending);
    this.pending = setTimeout(() => this.changed.fire(), 150);
  }

  async restart(): Promise<void> {
    await this.stop();
    await this.start();
  }

  summary(): Promise<Summary | undefined> {
    return this.request<Summary>("rosio/summary", {});
  }

  findings(): Promise<Finding[]> {
    return this.request<Finding[]>("rosio/findings", {}).then((rows) => rows ?? []);
  }

  entity(id: string): Promise<Entity | undefined> {
    return this.request<Entity | null>("rosio/entity", { id }).then((value) => value ?? undefined);
  }

  search(query: string, limit = 60): Promise<SearchHit[]> {
    return this.request<SearchHit[]>("rosio/search", { query, limit }).then((rows) => rows ?? []);
  }

  unresolved(): Promise<Unresolved[]> {
    return this.request<Unresolved[]>("rosio/unresolved", {}).then((rows) => rows ?? []);
  }

  children(): Promise<Record<string, string[]>> {
    return this.request<Record<string, string[]>>("rosio/children", {}).then((rows) => rows ?? {});
  }

  launchTree(): Promise<LaunchProfile[]> {
    return this.request<LaunchProfile[]>("rosio/launchTree", {}).then((rows) => rows ?? []);
  }

  frameTree(): Promise<FrameRow[]> {
    return this.request<FrameRow[]>("rosio/frameTree", {}).then((rows) => rows ?? []);
  }

  paramTree(profile: string): Promise<ParamTree> {
    return this.request<ParamTree>("rosio/paramTree", { profile }).then((tree) => ({
      profile: tree?.profile ?? profile,
      nodes: tree?.nodes ?? [],
    }));
  }

  graph(profile: string | undefined, focus?: string, depth?: number): Promise<GraphData> {
    return this.request<GraphData>("rosio/graph", { profile, focus, depth }).then((graph) => ({
      profile: graph?.profile ?? profile ?? null,
      nodes: graph?.nodes ?? [],
      channels: graph?.channels ?? [],
      links: graph?.links ?? [],
      truncated: graph?.truncated ?? false,
    }));
  }

  robotModel(profile: string | undefined, model: string | undefined): Promise<RobotModel> {
    return this.request<RobotModel>("rosio/robotModel", { profile, model }).then((robot) => ({
      profile: robot?.profile ?? profile ?? null,
      model: robot?.model ?? null,
      models: robot?.models ?? [],
      links: robot?.links ?? [],
      joints: robot?.joints ?? [],
      roots: robot?.roots ?? [],
      broadcasts: robot?.broadcasts ?? [],
      truncated: robot?.truncated ?? false,
      limit: robot?.limit ?? 0,
    }));
  }

  mesh(file: string): Promise<MeshData> {
    return this.request<MeshData>("rosio/mesh", { file }).then((mesh) => mesh ?? { file, format: "", refused: "file_missing" });
  }

  async extract(): Promise<void> {
    await this.client?.sendRequest("workspace/executeCommand", { command: "rosio-lsp.extract", arguments: [] });
  }

  private async request<T>(method: string, params: unknown): Promise<T | undefined> {
    if (!this.client?.isRunning()) return undefined;
    try {
      return await this.client.sendRequest<T>(method, params);
    } catch (error) {
      this.output.appendLine(`${method} failed: ${String(error)}`);
      if (method === "workspace/executeCommand") void vscode.window.showErrorMessage(String(error));
      return undefined;
    }
  }

  async stop(): Promise<void> {
    if (this.pending) clearTimeout(this.pending);
    this.pending = undefined;
    const client = this.client;
    this.client = undefined;
    if (client?.needsStop()) await client.stop().catch(() => undefined);
  }

  dispose(): void {
    void this.stop();
    this.changed.dispose();
    this.migrationChanged.dispose();
  }
}
