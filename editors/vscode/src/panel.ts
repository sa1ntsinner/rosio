// The atlas panel: one webview that shows the overview or one entity.
import * as path from "node:path";
import * as vscode from "vscode";
import type { ToHost, ToPanel } from "./panelMessages";
import type { Server } from "./server";

export class AtlasPanel implements vscode.Disposable {
  private panel: vscode.WebviewPanel | undefined;
  private showing: { type: "atlas" } | { type: "entity"; id: string } = { type: "atlas" };
  private ready = false;

  constructor(
    private readonly extensionUri: vscode.Uri,
    private readonly server: Server,
  ) {}

  async showAtlas(): Promise<void> {
    this.showing = { type: "atlas" };
    this.reveal();
    await this.update();
  }

  async showEntity(id: string): Promise<void> {
    this.showing = { type: "entity", id };
    this.reveal();
    await this.update();
  }

  /** Redraws what is showing, after the server loaded a new atlas. */
  async update(): Promise<void> {
    if (!this.panel || !this.ready) return;
    const findings = await this.server.findings();
    let message: ToPanel | undefined;
    if (this.showing.type === "entity") {
      const entity = await this.server.entity(this.showing.id);
      if (entity) message = { type: "entity", entity, findings, layers: (await this.server.summary())?.layers ?? [] };
      else this.showing = { type: "atlas" };
    }
    if (!message) {
      const summary = await this.server.summary();
      if (!summary) return;
      message = { type: "atlas", summary, findings, workspace: workspaceName() };
    }
    this.panel.title = message.type === "entity" ? message.entity.label : "rosio atlas";
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
    this.panel = vscode.window.createWebviewPanel("rosio.atlas", "rosio atlas", vscode.ViewColumn.Active, {
      enableScripts: true,
      retainContextWhenHidden: true,
      localResourceRoots: [media, out],
    });
    this.panel.iconPath = vscode.Uri.joinPath(media, "rosio.svg");
    this.panel.webview.html = this.html(this.panel.webview, media, out);
    this.panel.webview.onDidReceiveMessage((message: ToHost) => this.receive(message));
    this.panel.onDidDispose(() => {
      this.panel = undefined;
      this.ready = false;
    });
  }

  private async receive(message: ToHost): Promise<void> {
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
      case "extract":
        await vscode.commands.executeCommand("rosio.extract");
        break;
      case "find":
        await vscode.commands.executeCommand("rosio.find");
        break;
      case "atlas":
        await this.showAtlas();
        break;
      case "entity":
        await this.showEntity(message.id);
        break;
      case "graph":
        await vscode.commands.executeCommand("rosio.showGraph", message.id);
        break;
    }
  }

  private html(webview: vscode.Webview, media: vscode.Uri, out: vscode.Uri): string {
    const nonce = [...Array(32)].map(() => Math.floor(Math.random() * 36).toString(36)).join("");
    const css = webview.asWebviewUri(vscode.Uri.joinPath(media, "panel.css"));
    const script = webview.asWebviewUri(vscode.Uri.joinPath(out, "webview.js"));
    const policy = [
      "default-src 'none'",
      `style-src ${webview.cspSource}`,
      `script-src 'nonce-${nonce}'`,
      `img-src ${webview.cspSource}`,
    ].join("; ");
    return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta http-equiv="Content-Security-Policy" content="${policy}">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="${css}">
<title>rosio atlas</title>
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

function workspaceName(): string {
  const folder = vscode.workspace.workspaceFolders?.[0];
  return folder ? path.basename(folder.uri.fsPath) : "rosio";
}
