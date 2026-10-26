// Starts a real VS Code with the extension, the rosio language server and a fixture atlas.
//
// The language server comes from ROSIO_LSP, else from server/ inside the extension, which the
// extension finds by itself as a packaged one does, else from the cargo target directory of the
// repository. The workspace is the repository itself, so the fixture atlas resolves the
// sources it cites under tests/fixtures. ROSIO_E2E_EXTENSION names an installed extension
// directory to test instead of this source tree.
import { existsSync, rmSync, mkdirSync, mkdtempSync, writeFileSync } from "node:fs";
import { dirname, join, resolve } from "node:path";
import { fileURLToPath } from "node:url";
import { runTests } from "@vscode/test-electron";

const here = dirname(fileURLToPath(import.meta.url));
const source = resolve(here, "..");
const extension = process.env.ROSIO_E2E_EXTENSION ? resolve(process.env.ROSIO_E2E_EXTENSION) : source;
const repository = resolve(source, "..", "..");
const executable = process.platform === "win32" ? "rosio-lsp.exe" : "rosio-lsp";
const targets = [process.env.CARGO_TARGET_DIR, join(repository, "target")].filter(Boolean);
const bundled = !process.env.ROSIO_LSP && existsSync(join(extension, "server", executable));
const server = bundled
  ? undefined
  : (process.env.ROSIO_LSP ??
    targets.map((target) => join(target, "release", executable)).find((path) => existsSync(path)));

if (!bundled && !server) {
  console.error(
    `no ${executable}: build it with cargo build --release -p rosio-lsp, put it in server/, or set ROSIO_LSP`,
  );
  process.exit(2);
}
console.log(`extension: ${extension}, language server: ${bundled ? join("server", executable) : server}`);

const testRoot = join(source, ".vscode-test");
mkdirSync(testRoot, { recursive: true });
rmSync(join(testRoot, "migration-applied"), { force: true });
const scratch = mkdtempSync(join(testRoot, "rosio-e2e-"));
const workspace = join(scratch, "rosio.code-workspace");
writeFileSync(
  workspace,
  JSON.stringify({
    folders: [{ path: repository }],
    settings: {
      ...(server ? { "rosio.server.path": server } : {}),
      "rosio.command": [process.execPath, join(here, "migration-engine.mjs")],
      "rosio.atlasPath": "editors/vscode/test-e2e/fixture/atlas.json",
    },
  }),
);

await runTests({
  vscodeExecutablePath: process.env.VSCODE_EXECUTABLE_PATH,
  extensionDevelopmentPath: extension,
  extensionTestsPath: join(here, "suite", "index.cjs"),
  launchArgs: [
    workspace,
    "--disable-extensions",
    "--disable-workspace-trust",
    "--user-data-dir",
    join(scratch, "user"),
  ],
});
