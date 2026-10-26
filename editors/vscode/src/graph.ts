// The system graph: one webview that draws the nodes of a profile and the channels between them.
import * as vscode from "vscode";
import type { ToGraph, ToGraphHost } from "./graphMessages";
import { segments } from "./ids";
import type { Finding, GraphData } from "./protocol";
import type { Server } from "./server";

/** Two hops from a node reach the nodes on the other end of its channels. */
const FOCUS_DEPTH = 2;

export class GraphPanel implements vscode.Disposable {
  private panel: vscode.WebviewPanel | undefined;
  private ready = false;
  private profile: string | undefined;
  private focus: string | null = null;

  constructor(
    private readonly extensionUri: vscode.Uri,
    private readonly server: Server,
  ) {}

  async show(focus: string | null = null): Promise<void> {
    this.focus = focus;
    this.reveal();
    await this.update();
  }

  async update(): Promise<void> {
    if (!this.panel || !this.ready) return;
    const profiles = (await this.server.summary())?.profiles ?? [];
    if (!this.profile || !profiles.includes(this.profile)) this.profile = profiles[0];
    const graph = await this.server.graph(this.profile, this.focus ?? undefined, this.focus ? FOCUS_DEPTH : undefined);
    const row = this.focus ? [...graph.nodes, ...graph.channels].find((item) => item.id === this.focus) : undefined;
    const message: ToGraph = { type: "graph", graph, profiles, focus: row ? { id: row.id, label: row.label } : null, findings: await this.findings(graph) };
    this.panel.title = row ? `Graph: ${row.label}` : "System graph";
    await this.panel.webview.postMessage(message);
  }

  private async findings(graph: GraphData): Promise<Record<string, Finding[]>> {
    const flagged = new Set([...graph.nodes, ...graph.channels].filter((row) => row.findings > 0).map((row) => row.id));
    if (!flagged.size) return {};
    const about: Record<string, Finding[]> = {};
    for (const finding of await this.server.findings()) {
      // a finding id is find:<rule>|<subject>|<discriminator>
      const subject = segments(finding.id)[1];
      if (flagged.has(subject)) (about[subject] ??= []).push(finding);
    }
    return about;
  }

  private reveal(): void {
    if (this.panel) {
      this.panel.reveal(undefined, false);
      return;
    }
    const media = vscode.Uri.joinPath(this.extensionUri, "media");
    const out = vscode.Uri.joinPath(this.extensionUri, "out");
    this.ready = false;
    this.panel = vscode.window.createWebviewPanel("rosio.graph", "System graph", vscode.ViewColumn.Active, {
      enableScripts: true,
      retainContextWhenHidden: true,
      localResourceRoots: [media, out],
    });
    this.panel.iconPath = vscode.Uri.joinPath(media, "rosio.svg");
    this.panel.webview.html = this.html(this.panel.webview, media, out);
    this.panel.webview.onDidReceiveMessage((message: ToGraphHost) => this.receive(message));
    this.panel.onDidDispose(() => {
      this.panel = undefined;
      this.ready = false;
    });
  }

  private async receive(message: ToGraphHost): Promise<void> {
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
      case "focus":
        this.focus = message.id;
        await this.update();
        break;
      case "profile":
        this.profile = message.profile;
        await this.update();
        break;
    }
  }

  private html(webview: vscode.Webview, media: vscode.Uri, out: vscode.Uri): string {
    const nonce = [...Array(32)].map(() => Math.floor(Math.random() * 36).toString(36)).join("");
    const css = webview.asWebviewUri(vscode.Uri.joinPath(media, "graph.css"));
    const script = webview.asWebviewUri(vscode.Uri.joinPath(out, "graph.js"));
    const policy = ["default-src 'none'", `style-src ${webview.cspSource}`, `script-src 'nonce-${nonce}'`].join("; ");
    return `<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta http-equiv="Content-Security-Policy" content="${policy}">
<meta name="viewport" content="width=device-width, initial-scale=1">
<link rel="stylesheet" href="${css}">
<title>System graph</title>
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
