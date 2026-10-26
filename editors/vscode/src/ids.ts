// Reading atlas ids for display. The grammar is in AGENTS.md section 3.1.

export const KIND_NAMES: Record<string, string> = {
  profile: "Profiles",
  node: "Nodes",
  topic: "Topics",
  service: "Services",
  action: "Actions",
  param: "Parameters",
  frame: "Frames",
  launchfile: "Launch files",
  pkg: "Packages",
  plugin: "Plugins",
  bag: "Recordings",
};

export function kindOf(id: string): string {
  return id.slice(0, id.indexOf(":"));
}

export function segments(id: string): string[] {
  const body = id.slice(id.indexOf(":") + 1);
  return body.split("|").map((part) => {
    try {
      return decodeURIComponent(part);
    } catch {
      return part;
    }
  });
}

/** The short name a person recognises: the topic, the node, the parameter key. */
export function labelOf(id: string): string {
  const parts = segments(id);
  if (parts[0]?.startsWith("~")) return `${parts[1] ?? parts[0]} (unnamed)`;
  return parts[parts.length - 1] ?? id;
}

/** What the label alone leaves out: the owner of a parameter, the package of a file. */
export function describeId(id: string): string {
  const parts = segments(id);
  switch (kindOf(id)) {
    case "param":
    case "file":
    case "launchfile":
      return parts.length > 1 ? parts[0] : "";
    default:
      return "";
  }
}
