// The Migration view as plain data, so its shape is tested without VS Code.
import type { BuildError, BuildPackage, MigrationBuild, MigrationChange, MigrationPlan } from "./protocol";

export const MIGRATION_EMPTY = 'Run "Plan migration" to list release changes.';
export const RELEASES = ["humble", "iron", "jazzy", "kilted", "lyrical"];

export type Section = { kind: "hand" | "automatic"; label: string; count: number; packages: PackageGroup[] };
export type PackageGroup = { section: Section["kind"]; package: string; changes: MigrationChange[] };

export function targetReleases(from?: string): string[] {
  const index = from ? RELEASES.indexOf(from) : -1;
  return RELEASES.slice(index + 1);
}

/** By hand first: those are the changes a person still has to make. */
export function migrationSections(plan?: MigrationPlan): Section[] {
  const changes = plan?.changes ?? [];
  const sections: Section[] = [
    { kind: "hand", label: "By hand", count: 0, packages: [] },
    { kind: "automatic", label: "Exact rewrites", count: 0, packages: [] },
  ];
  for (const section of sections) {
    const rows = changes.filter((c) => (c.replacement === undefined) === (section.kind === "hand"));
    section.count = rows.length;
    section.packages = migrationGroups({ changes: rows } as MigrationPlan).map((group) => ({
      section: section.kind,
      package: group.package,
      changes: group.changes,
    }));
  }
  return sections.filter((section) => section.count > 0);
}

export function migrationGroups(plan?: MigrationPlan): { package: string; changes: MigrationChange[]; count: number }[] {
  const groups = new Map<string, MigrationChange[]>();
  for (const change of plan?.changes ?? []) {
    const rows = groups.get(change.package) ?? [];
    rows.push(change);
    groups.set(change.package, rows);
  }
  return [...groups].sort(([a], [b]) => a.localeCompare(b)).map(([name, changes]) => ({
    package: name,
    count: changes.length,
    changes: changes.sort((a, b) => a.file.localeCompare(b.file) || a.line - b.line || a.column - b.column || a.rule.localeCompare(b.rule)),
  }));
}

/** A row names what changed; the sentence that says what to do goes in the tooltip. */
export function changeTitle(change: MigrationChange): string {
  return change.title ?? change.say.split("; ")[0];
}

export function changeWhere(change: MigrationChange): string {
  const name = change.file.split("/").pop() ?? change.file;
  return `${name}:${change.line}`;
}

/** Where it is, and, for the few that only warn, that they do: the rest stop the build. */
export function changeDescription(change: MigrationChange): string {
  return change.effect === "deprecated" ? `${changeWhere(change)}  warns only` : changeWhere(change);
}

/** The line read first: how much is left to a person, and where the move lands. */
export function planMessage(plan?: MigrationPlan): string | undefined {
  if (!plan) return undefined;
  const hand = plan.changes.filter((c) => c.replacement === undefined).length;
  const automatic = plan.changes.length - hand;
  if (!plan.changes.length) return `Nothing in this workspace changes from ${plan.from} to ${plan.to}.`;
  const parts = [];
  if (hand) parts.push(`${hand} ${hand === 1 ? "change needs" : "changes need"} you`);
  if (automatic) parts.push(`${automatic} ${automatic === 1 ? "is an exact rewrite" : "are exact rewrites"}`);
  return `${parts.join(", ")}.`;
}

export type BuildSection = { kind: "build"; label: string; description: string; ok: boolean; packages: BuildPackage[] };

/** Only what did not build: a package that built has nothing to open. */
export function buildSection(build?: MigrationBuild): BuildSection | undefined {
  if (!build) return undefined;
  const sources = build.packages.filter((p) => !p.name.startsWith("@"));
  const built = sources.filter((p) => p.status === "finished").length;
  const order = { failed: 0, skipped: 1, finished: 2 };
  return {
    kind: "build",
    label: `Build in ${build.to}`,
    description: `${built} of ${sources.length} built`,
    ok: build.ok,
    packages: build.packages
      .filter((p) => p.status !== "finished")
      .sort((a, b) => order[a.status] - order[b.status] || a.name.localeCompare(b.name)),
  };
}

export function buildPackageLabel(name: string): string {
  return { "@setup": "Container setup", "@build": "Build" }[name] ?? name;
}

export function buildPackageDescription(row: BuildPackage): string {
  if (row.status === "skipped") return "not built";
  const missing = row.errors.filter((e) => e.status === "not_listed").length;
  const count = `${row.errors.length} ${row.errors.length === 1 ? "error" : "errors"}`;
  return missing ? `${count}, ${missing} not in the plan` : count;
}

export function buildErrorTitle(error: BuildError): string {
  const first = error.message.split("\n")[0].trim();
  return first.length > 120 ? `${first.slice(0, 119)}…` : first;
}

export function buildErrorWhere(error: BuildError): string {
  return error.file ? `${error.file.split("/").pop()}:${error.line}` : "no line";
}

/** Errors the plan does not list come first: those are the ones nobody has a fix for yet. */
export function buildErrors(row: BuildPackage): BuildError[] {
  return [...row.errors].sort((a, b) => Number(a.status === "listed") - Number(b.status === "listed"));
}

/** The line read first once the build ran: what fails, and whether the plan already knows why. */
export function buildMessage(build?: MigrationBuild): string | undefined {
  if (!build) return undefined;
  const sources = build.packages.filter((p) => !p.name.startsWith("@"));
  const built = sources.filter((p) => p.status === "finished").length;
  if (build.ok) return `Every package builds in ${build.to}.`;
  const errors = build.packages.flatMap((p) => p.errors);
  const missing = errors.filter((e) => e.status === "not_listed").length;
  const failed = sources.length - built;
  const head = `${failed} of ${sources.length} ${sources.length === 1 ? "package does" : "packages do"} not build in ${build.to}`;
  if (!errors.length) return `${head}.`;
  if (!missing) return `${head}; the plan lists every error.`;
  return `${head}; ${missing} ${missing === 1 ? "error is" : "errors are"} not in the plan.`;
}
