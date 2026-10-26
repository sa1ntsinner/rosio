// Bundles the extension host code and the webview scripts.
import { build, context } from "esbuild";

const minify = process.argv.includes("--minify");
const watch = process.argv.includes("--watch");

const targets = [
  {
    entryPoints: ["src/extension.ts"],
    outfile: "out/extension.js",
    platform: "node",
    format: "cjs",
    target: "node20",
    external: ["vscode"],
  },
  {
    entryPoints: ["webview/main.ts"],
    outfile: "out/webview.js",
    platform: "browser",
    format: "iife",
    target: "es2022",
  },
  {
    entryPoints: ["webview/graph.ts"],
    outfile: "out/graph.js",
    platform: "browser",
    format: "iife",
    target: "es2022",
  },
  {
    entryPoints: ["webview/robot.ts"],
    outfile: "out/robot.js",
    platform: "browser",
    format: "iife",
    target: "es2022",
  },
];

for (const options of targets) {
  const full = { ...options, bundle: true, minify, sourcemap: !minify, logLevel: "warning" };
  if (watch) {
    const ctx = await context(full);
    await ctx.watch();
  } else {
    await build(full);
  }
}
