// The robot model: one webview that draws the links of a URDF model where its joints put them.
import * as vscode from "vscode";
import type { MeshData } from "./protocol";
import type { ToRobot, ToRobotHost } from "./robotMessages";
import type { Server } from "./server";

export class RobotPanel implements vscode.Disposable {
  private panel: vscode.WebviewPanel | undefined;
  private ready = false;
  private profile: string | undefined;
  private model: string | undefined;
  /** Mesh answers by file id, kept until the atlas is reloaded. */
  private meshes = new Map<string, MeshData>();

  constructor(
    private readonly extensionUri: vscode.Uri,
    private readonly server: Server,
  ) {}

  async show(): Promise<void> {
    this.reveal();
    await this.update();
  }

  /** `reloaded` is true when the atlas changed, which is when a mesh on disk may have changed too. */
  async update(reloaded = false): Promise<void> {
    if (reloaded) this.meshes.clear();
    if (!this.panel || !this.ready) return;
    const profiles = (await this.server.summary())?.profiles ?? [];
    if (this.profile && !profiles.includes(this.profile)) this.profile = undefined;
    let robot = await this.server.robotModel(this.profile, this.model);
    if (!this.model && !robot.links.length) {
      // a file of shared materials or macros is a model with no links; open the first one that has some
      for (const other of robot.models.filter((model) => model.id !== robot.model)) {
        const next = await this.server.robotModel(this.profile, other.id);
        if (next.links.length) {
          robot = next;
          break;
        }
      }
    }
    this.model = robot.model ?? undefined;
    const label = robot.models.find((model) => model.id === robot.model)?.robotName;
    this.panel.title = label ? `Robot: ${label}` : "Robot model";
    const files = new Set<string>();
    for (const link of robot.links) {
      for (const shape of [...(link.visuals ?? []), ...(link.collisions ?? [])]) {
        if (shape.geometry?.file) files.add(shape.geometry.file);
      }
    }
    for (const file of [...files].sort()) {
      if (!this.meshes.has(file)) this.meshes.set(file, await this.server.mesh(file));
    }
    const meshes = Object.fromEntries([...files].sort().map((file) => [file, this.meshes.get(file)!]));
    const message: ToRobot = { type: "robot", robot, profiles, meshes };
    await this.panel.webview.postMessage(message);
  }

  private reveal(): void {
    if (this.panel) {
      this.panel.reveal(undefined, false);
      return;
    }
    const media = vscode.Uri.joinPath(this.extensionUri, "media");
    const out = vscode.Uri.joinPath(this.extensionUri, "out");
    this.ready = false;
    this.panel = vscode.window.createWebviewPanel("rosio.robot", "Robot model", vscode.ViewColumn.Active, {
      enableScripts: true,
      retainContextWhenHidden: true,
      localResourceRoots: [media, out],
    });
    this.panel.iconPath = vscode.Uri.joinPath(media, "rosio.svg");
    this.panel.webview.html = this.html(this.panel.webview, media, out);
    this.panel.webview.onDidReceiveMessage((message: ToRobotHost) => this.receive(message));
    this.panel.onDidDispose(() => {
      this.panel = undefined;
      this.ready = false;
    });
  }

  private async receive(message: ToRobotHost): Promise<void> {
    switch (message.type) {
      case "ready":
        this.ready = true;
        await this.update();
        break;
      case "open":
        await vscode.window.showTextDocument(vscode.Uri.parse(message.uri), {
          selection: new vscode.Range(message.line, 0, message.line, 0),
          viewColumn: vscode.ViewColumn.Beside,
        });
        break;
      case "entity":
        await vscode.commands.executeCommand("rosio.showEntity", message.id);
        break;
      case "model":
        this.model = message.model;
        await this.update();
        break;
      case "profile":
        this.profile = message.profile || undefined;
        this.model = undefined;
        await this.update();
        break;
    }
  }

  private html(webview: vscode.Webview, media: vscode.Uri, out: vscode.Uri): string {
    const nonce = [...Array(32)].map(() => Math.floor(Math.random() * 36).toString(36)).join("");
    const base = webview.asWebviewUri(vscode.Uri.joinPath(media, "graph.css"));
    const css = webview.asWebviewUri(vscode.Uri.joinPath(media, "robot.css"));
    const script = webview.asWebviewUri(vscode.Uri.joinPath(out, "robot.js"));
    const policy = ["default-src 'none'", `style-src ${webview.cspSource}`, `script-src 'nonce-${nonce}'`].join("; ");
    return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta http-equiv="Content-Security-Policy" content="${policy}">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="${base}">
<link rel="stylesheet" href="${css}">
<title>Robot model</title>
</head>
<body><main><p class="muted">Reading the atlas</p></main>
<script nonce="${nonce}" src="${script}"></script>
</body>
</html>`;
  }

  dispose(): void {
    this.panel?.dispose();
  }
}
