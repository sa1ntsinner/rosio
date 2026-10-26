import { existsSync } from "node:fs";
import { join } from "node:path";

/** The rosio-lsp to start: the setting when set, then the binary inside the extension, then PATH. */
export function serverCommand(
  setting: string | undefined,
  extensionPath: string,
  platform: string = process.platform,
  exists: (path: string) => boolean = existsSync,
): string {
  if (setting?.trim()) return setting.trim();
  const name = platform === "win32" ? "rosio-lsp.exe" : "rosio-lsp";
  const bundled = join(extensionPath, "server", name);
  return exists(bundled) ? bundled : "rosio-lsp";
}
