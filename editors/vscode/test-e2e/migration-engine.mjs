import { existsSync, writeFileSync } from "node:fs";
import { join } from "node:path";

const state = join(process.cwd(), "editors/vscode/.vscode-test/migration-applied");
const changes = [
  { rule: "fixture.automatic", package: "fixture", file: "editors/vscode/test-e2e/fixture/upgrade/CMakeLists.txt", line: 1, column: 4, found: "old", effect: "removed", release: "lyrical", say: "Use new", source: "https://example.org/migration", replacement: "new", works_from: "humble" },
  { rule: "fixture.manual", package: "fixture", file: "editors/vscode/test-e2e/fixture/upgrade/CMakeLists.txt", line: 2, column: 1, found: "manual", effect: "deprecated", release: "jazzy", say: "Review this call", source: "https://example.org/migration" },
];
const apply = process.argv.includes("apply");
if (apply) writeFileSync(state, "applied");
const remaining = existsSync(state) && !apply ? changes.slice(1) : changes;
console.log(JSON.stringify({ ok: true, verb: apply ? "migrate apply" : "migrate plan", data: {
  from: "humble", to: "lyrical", summary: { changes: remaining.length, packages: 1, automatic: remaining.length - 1, by_hand: 1 }, changes: remaining,
  ...(apply ? { diff: "editors/vscode/.vscode-test/migration.diff" } : {}),
} }));
