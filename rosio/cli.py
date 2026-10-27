"""Command line entry point. Argparse only, no third party CLI framework.

Every verb except `extract` reads one atlas.json off disk. `probe` reads one, adds the
observed layer to it and writes it back. Nothing here re extracts to answer a question,
because that is how an agent uses this: one extraction, then many small questions against
the file it wrote.

Two exit code rules hold across the query verbs. A question that was not answered, an
ambiguous selector or a name that is in no atlas, exits with the usage code, so a caller can
branch on it without parsing prose. The drift gate is the one exception and has its own
code, because a build that fails on a finding is not a usage mistake.

`probe` refuses to run without a domain id it was given or that the caller already exported.
A machine that runs this may be attached to a vehicle, the middleware has no idea which
domain a caller meant, and a probe that discovers the wrong one is the worst thing this tool
can do. The domain is exported to the child after the setup files for the same reason: a
workspace setup file that exports its own would otherwise win.
"""

import argparse
import os
import sys
from collections.abc import Callable, Sequence

from rosio import __version__
from rosio.errors import EXIT_OK, EXIT_USAGE, RosioError, UsageError, exit_code_for
from rosio.render import plural

PROG = "rosio"

DEFAULT_DETERMINISTIC = True
"""Output is byte stable unless the command line asks for a clock."""

DESCRIPTION = (
    "Build a four layer atlas of a ROS 2 system (declared, coded, observed, recorded) "
    "and report the drift between the layers. Extraction is deterministic."
)

EPILOG = (
    "Outputs land under --out: atlas.json (machine first), ATLAS_REPORT.md, DRIFT.md, atlas.html.\n"
    "Start with: rosio init <workspace>, then rosio extract <workspace>, then rosio drift,\n"
    "then rosio render.\n"
    "Add a recording with extract --bag DIR, and a running system with rosio probe.\n"
    "Every other verb reads atlas.json and never extracts again: pass --atlas FILE or --out DIR.\n"
    "Exit codes: 0 answered, 1 stamps found an error, 2 not answered or bad usage,\n"
    "3 a collector reached nothing, 4 contract, 5 the drift gate tripped."
)

DRIFT_CAPABILITY = "drift.findings"
"""The feature probe that says the rules ran over an atlas. An atlas without it holds no
findings, and reporting none for it would read exactly like a clean stack."""

WAIVERS_KEY = "waivers"
"""The configuration key and the flag name. A waiver is read here and never during extract:
`atlas.json` carries every finding whatever a repository has accepted, so a machine with a
waiver file and a machine without one write the same bytes."""

VERBS = (
    "extract",
    "probe",
    "drift",
    "explain",
    "param",
    "query",
    "path",
    "diff",
    "unresolved",
    "render",
    "install",
    "uninstall",
    "serve",
    "version",
    "hook",
    "stamps",
    "init",
    "bundle",
    "migrate",
    "graph",
)
"""The twenty verbs. `graph` gives a terminal, a CI job or an agent without MCP the answer of
the MCP graph tool, and draws it as Graphviz or Mermaid. `migrate` moves a workspace between
ROS 2 releases. `bundle` carries an
atlas and the files its findings cite to someone on another machine. `init` writes the
configuration extraction would fall back to as a `rosio.yaml` to edit. `hook` manages the
git side of keeping an atlas current (the hooks, the merge driver and the rebuild). `stamps`
checks the header stamps of one recording and needs neither a workspace nor a ROS
installation, which is why it is a verb of its own and not a flag of `extract`."""

CONTRACT_VERB_RULE = (
    "a verb exists when it is in this list, adding one is a line here in the same commit "
    "as the code with the reason beside it"
)
"""The sentence of AGENTS section 11 this set lives under, held equal to the contract by a
test. The set is not closed. What stood here before said section 11 had closed it at
fourteen, which section 11 does not say and the fifteenth verb had already disproved."""


DOMAIN_ENV = "ROS_DOMAIN_ID"
LOCALHOST_ENV = "ROS_LOCALHOST_ONLY"

SERVE_PORT = 8787
SERVE_PATH = "/mcp"
"""The http defaults, restated here so building the parser does not import the server and
everything it loads. A test holds them equal to the server's own."""


class _Version(argparse.Action):
    """`--version`, saying what `rosio version` says. The schema is read when the flag is
    given, so building the parser imports no model."""

    def __init__(self, option_strings, **named):
        named.update(dest=argparse.SUPPRESS, default=argparse.SUPPRESS, nargs=0)
        super().__init__(option_strings, **named)

    def __call__(self, parser, *_given):
        from rosio.model import SCHEMA_VERSION

        print(f"{PROG} {__version__}, atlas schema {SCHEMA_VERSION}")
        parser.exit()


PROFILES_SHOWN = 8
"""How many profile names the extract summary prints before it counts the rest."""


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog=PROG,
        description=DESCRIPTION,
        epilog=EPILOG,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--version", action=_Version, help="print the version and the atlas schema version"
    )
    _add_global_flags(parser, root=True)

    subparsers = parser.add_subparsers(dest="command", metavar="command")

    init = _sub(
        subparsers,
        "init",
        "write a rosio.yaml to edit",
        "Write the configuration extraction would use without one: the roots found in the "
        "workspace, the ROS setup files a launch file needs, and one profile per launch file. "
        "Edit it so each profile is one system you actually start.",
    )
    init.add_argument(
        "workspace",
        nargs="?",
        default=".",
        metavar="PATH",
        help="workspace directory, default is the current one",
    )
    init.add_argument("--force", action="store_true", help="replace an existing rosio.yaml")
    init.add_argument(
        "--print", action="store_true", help="print the configuration instead of writing it"
    )
    init.add_argument(
        "--package",
        action="append",
        metavar="NAME",
        help="write the profiles of this package only, repeatable, for an install tree "
        "such as /opt/ros/<distro> that holds hundreds of launch files",
    )
    init.add_argument(
        "--config",
        metavar="FILE",
        help="file to write, default is rosio.yaml in PATH, with paths below its directory "
        "written relative to it",
    )

    extract = _sub(
        subparsers,
        "extract",
        "read a workspace and write the declared and coded layers",
        "Parse launch files, yaml parameter profiles, urdf and xacro, interface definitions "
        "and pluginlib manifests, then read the C++ and Python sources for what the code "
        "actually publishes, subscribes, declares and broadcasts. The drift rules run over "
        "the result, so atlas.json carries the findings and rosio drift reports them.",
    )
    extract.add_argument(
        "workspace",
        nargs="?",
        default=".",
        metavar="PATH",
        help="workspace directory, its src tree or a single package, default is the current one",
    )
    extract.add_argument(
        "--layer",
        action="append",
        choices=["declared", "coded"],
        help="restrict extraction to one layer, repeatable",
    )
    extract.add_argument(
        "--launch-sandbox",
        choices=("auto", "required", "off"),
        help="isolate launch execution with bwrap when available (auto), require it, "
        "or turn it off",
    )
    extract.add_argument(
        "--install-tree",
        metavar="DIR",
        help="built install tree to read instead of the one beside the sources, "
        "replacing any the config declares and taking the overlay setup file with it",
    )
    extract.add_argument(
        "--profile",
        action="append",
        metavar="NAME",
        help="profile slug to resolve, repeatable, default is every profile in the config",
    )
    extract.add_argument(
        "--config",
        metavar="FILE",
        help="configuration file. default: rosio.yaml in the workspace or the directory above "
        "it, then in the working directory or the directory above it, then one profile per "
        "launch file. the summary names the file it read",
    )
    extract.add_argument(
        "--collector",
        action="append",
        metavar="NAME",
        help="run a single collector, repeatable, default is the committed repository set. "
        "the live probe is not selectable here, it is the probe verb",
    )
    extract.add_argument(
        "--bag",
        action="append",
        metavar="PATH",
        help="rosbag2 directory to read for the recorded layer, repeatable, "
        "a directory of recordings reads every recording under it",
    )
    extract.add_argument(
        "--no-runtime",
        action="store_true",
        help="read the repository only, ignoring the bag roots the config names",
    )
    extract.add_argument(
        "--fail-on-violation",
        action="store_true",
        help="exit non zero when a run recorded a problem, not only when the atlas is invalid",
    )

    probe = _sub(
        subparsers,
        "probe",
        "attach to a live system and add the observed layer to an atlas",
        "Run a short rclpy probe: node names, endpoints with their QoS, effective parameter "
        "values, the TF tree, publication rates and lifecycle state, plus the argv of every "
        "running ROS process. Needs a sourced ROS 2 environment. The result is written into "
        "the atlas named by --atlas, or into a new one when there is none, and nothing is "
        "extracted again. A domain id is mandatory: pass --domain-id or export "
        f"{DOMAIN_ENV} first, because probing the wrong domain on a machine attached to a "
        "vehicle is not acceptable.",
    )
    probe.add_argument(
        "workspace",
        nargs="?",
        default=".",
        metavar="PATH",
        help="workspace the atlas describes, default is the current directory",
    )
    probe.add_argument(
        "--stamps",
        action="store_true",
        help="check live header stamps; use several seconds (--duration 10) for useful checks",
    )
    probe.add_argument(
        "--duration",
        type=float,
        default=10.0,
        metavar="SECONDS",
        help="sampling window in seconds, default 10",
    )
    probe.add_argument(
        "--settle",
        type=float,
        default=None,
        metavar="SECONDS",
        help="discovery time before anything is read, default 2",
    )
    probe.add_argument(
        "--domain-id",
        metavar="N",
        help=f"ROS domain to attach to, mandatory unless {DOMAIN_ENV} is already exported",
    )
    probe.add_argument(
        "--localhost-only",
        action="store_true",
        help=f"set {LOCALHOST_ENV}=1 for the probe, so it cannot reach another machine",
    )
    probe.add_argument(
        "--rmw",
        action="append",
        metavar="NAME",
        help="middleware to probe, repeatable; default detects process environments; "
        "use default to unset RMW_IMPLEMENTATION",
    )
    probe.add_argument(
        "--ros-setup",
        action="append",
        metavar="FILE",
        help="setup file the probe child sources, repeatable, default is the configured one",
    )
    probe.add_argument(
        "--profile",
        metavar="NAME",
        help="file the observation against one profile, default is against every profile",
    )
    probe.add_argument(
        "--config",
        metavar="FILE",
        help="configuration file, default is rosio.yaml beside the workspace if there is one",
    )
    probe.add_argument(
        "--atlas",
        metavar="FILE",
        help="atlas to add the observed layer to, a directory holding one, "
        "default is <out>/atlas.json and a new atlas when there is none",
    )

    drift = _sub(
        subparsers,
        "drift",
        "report what the layers disagree about",
        "Print the findings the atlas carries, ranked, with the file and line to edit: "
        "parameters set twice, plugins that load nothing, frames with no broadcaster, remaps "
        "that go nowhere. The rules run during extract, so this reads atlas.json and grades "
        "it. --min-severity filters what is printed and --fail-on decides the exit code, "
        "because the print threshold and the CI gate are two decisions. --waivers is the "
        "third: a finding this repository has accepted is still printed, with the reason, "
        "and stops being graded.",
        flags=READ_FLAGS,
    )
    drift.add_argument(
        "--fail-on",
        choices=["error", "warn", "info", "never"],
        default="never",
        help="lowest severity that makes the command exit non zero, default never",
    )
    drift.add_argument(
        "--min-severity",
        choices=["error", "warn", "info"],
        default="info",
        help="lowest severity to print, default info",
    )
    drift.add_argument(
        "--only",
        action="append",
        metavar="RULE",
        help="report a single drift rule, repeatable",
    )
    drift.add_argument(
        "--waivers",
        action="append",
        metavar="FILE",
        help="findings this repository has accepted, repeatable, "
        f"default is the {WAIVERS_KEY} key of the configuration",
    )
    drift.add_argument(
        "--config",
        metavar="FILE",
        help=f"configuration file to read the {WAIVERS_KEY} key from, "
        "default is rosio.yaml here or one directory up",
    )
    _add_atlas_flags(drift)

    explain = _sub(
        subparsers,
        "explain",
        "explain one drift finding or one entity in full",
        "Print the evidence behind a single finding: the file and line of the declaration, the "
        "source site in the code, and what the live system reported. An entity selector prints "
        "that entity in full instead, and a rule id lists that rule's findings.",
        flags=READ_FLAGS,
    )
    explain.add_argument(
        "selector",
        nargs="?",
        metavar="SELECTOR",
        help="finding fingerprint or id as drift prints it, a rule id such as "
        "param.shadowed, or an entity such as /planner_server",
    )
    _add_atlas_flags(explain)

    param = _sub(
        subparsers,
        "param",
        "show the candidate chain behind a parameter",
        "Print every candidate for a parameter in order, which one wins, which mechanism set "
        "it and which file to edit, plus the ones that lost and why.",
        flags=READ_FLAGS,
    )
    param.add_argument(
        "selector",
        nargs="*",
        metavar="SELECTOR",
        help="node and name, for example /planner_server GridBased.cost_penalty",
    )
    param.add_argument(
        "--profile", metavar="NAME", help="restrict to one profile, which also picks the node"
    )
    _add_atlas_flags(param)

    query = _sub(
        subparsers,
        "query",
        "look up entities in the atlas",
        "Answer one question against atlas.json, for example every subscriber of a topic or "
        "every parameter a node declares. Matched by name and kind, no language model. "
        "Machine readable with --json.",
        flags=READ_FLAGS,
    )
    query.add_argument(
        "question",
        nargs="*",
        metavar="WORDS",
        help='for example topic:/scan, or "which nodes publish cmd_vel"',
    )
    _add_atlas_flags(query)

    path = _sub(
        subparsers,
        "path",
        "trace how data gets from one entity to another",
        "Walk the graph from a source to a sink through topics, services, remaps and TF, and "
        "show every hop with the direction it is read in. A route through lifecycle "
        "management or launch is shown only when no data route exists, and says so.",
        flags=READ_FLAGS,
    )
    path.add_argument(
        "source", nargs="?", metavar="SOURCE", help="where to start, a node, topic or frame name"
    )
    path.add_argument(
        "target", nargs="?", metavar="TARGET", help="where to end, a node, topic or frame name"
    )
    path.add_argument(
        "--max-hops", type=int, default=12, metavar="N", help="give up after N hops, default 12"
    )
    _add_atlas_flags(path)

    diff = _sub(
        subparsers,
        "diff",
        "compare two atlases, or two profiles of one",
        "Diff two atlas.json files and report what appeared, disappeared or changed. Useful in "
        "review to see the graph effect of a branch. Two --profile values compare two profiles "
        "instead, which is how sim and real are told apart.",
        flags=READ_FLAGS,
    )
    diff.add_argument(
        "before",
        nargs="?",
        metavar="BEFORE",
        help="the older atlas.json, or a directory holding one",
    )
    diff.add_argument(
        "after", nargs="?", metavar="AFTER", help="the newer atlas.json, or a directory holding one"
    )
    diff.add_argument(
        "--profile",
        action="append",
        metavar="NAME",
        help="compare two profiles, given exactly twice",
    )
    diff.add_argument(
        "--rev",
        action="append",
        metavar="REV",
        help="build two git revisions of one workspace and compare them, given exactly twice",
    )
    diff.add_argument("--repo", metavar="DIR", help="repository the revisions are read from")
    diff.add_argument(
        "--cache-rules",
        action="store_true",
        help="with --rev, also print what the build cache keys on and what it cannot know",
    )
    diff.add_argument(
        "--workspace",
        metavar="PATH",
        help="workspace inside the revision, default src when there is one",
    )
    _add_atlas_flags(diff)

    unresolved = _sub(
        subparsers,
        "unresolved",
        "list what extraction could not resolve",
        "Print every blind spot with its site, the raw expression and what was wanted, so a "
        "reader can tell a hollow extraction from a complete one.",
        flags=READ_FLAGS,
    )
    from rosio.model import Want

    unresolved.add_argument(
        "--want",
        action="append",
        metavar="WANT",
        help="restrict to one want, repeatable: " + ", ".join(item.value for item in Want),
    )
    _add_atlas_flags(unresolved)

    graph = _sub(
        subparsers,
        "graph",
        "show who talks to what, as text or as a diagram",
        "Nodes and the topics, services and actions between them, with the QoS the atlas "
        "states and the rule ids of findings. text is the answer of the MCP graph tool, dot a "
        "Graphviz digraph and mermaid a Mermaid flowchart, data flowing from publisher or "
        "caller to subscriber or server.",
        flags=READ_FLAGS,
    )
    graph.add_argument("--profile", metavar="NAME", help="restrict to one profile")
    graph.add_argument(
        "--focus",
        metavar="NODE_OR_CHANNEL",
        help="only the neighbourhood of one node, topic, service or action",
    )
    graph.add_argument(
        "--depth", type=int, default=2, metavar="N", help="hops around --focus, default 2"
    )
    graph.add_argument(
        "--wrappers",
        action="store_true",
        help="include library endpoints such as tf2_ros listeners and diagnostic_updater",
    )
    graph.add_argument(
        "--format", choices=("text", "dot", "mermaid"), default="text", help="default text"
    )
    graph.add_argument(
        "--atlas",
        metavar="FILE",
        help="atlas.json to read, a directory holding one, default is <out>/atlas.json",
    )
    graph.add_argument(
        "--limit",
        type=int,
        metavar="N",
        help="channels shown, default 8 for text and every channel for a diagram",
    )

    render = _sub(
        subparsers,
        "render",
        "write the human readable outputs from an existing atlas",
        "Render ATLAS_REPORT.md, DRIFT.md and the single file atlas.html from atlas.json. The "
        "html template is vendored, nothing is fetched at run time.",
        flags=READ_FLAGS,
    )
    render.add_argument(
        "--format",
        action="append",
        choices=["md", "html"],
        help="restrict the output: md writes ATLAS_REPORT.md and DRIFT.md, html writes "
        "atlas.html, repeatable, default is all three files",
    )
    _add_atlas_flags(render)

    install = _sub(
        subparsers,
        "install",
        "register rosio with a coding agent",
        "Write the agent side glue (tool or command definitions) so an agent can call the "
        "subcommands and read atlas.json without further setup.",
        flags=(),
    )
    from rosio.agent.platforms import keys as platform_keys

    assistants = ", ".join(platform_keys())
    install.add_argument(
        "target", nargs="?", metavar="TARGET", help=f"assistant to register with: {assistants}"
    )
    install.add_argument("--list", action="store_true", help="print the platform table and exit")
    _add_agent_flags(install)
    install.add_argument("--force", action="store_true", help="replace content rosio did not write")

    uninstall = _sub(
        subparsers,
        "uninstall",
        "remove the agent registration written by install",
        "Undo install. Leaves the atlas outputs alone.",
        flags=(),
    )
    uninstall.add_argument(
        "target", nargs="?", metavar="TARGET", help=f"assistant to remove: {assistants}"
    )
    _add_agent_flags(uninstall)
    uninstall.add_argument(
        "--force", action="store_true", help="delete a managed file rosio did not write"
    )

    serve = _sub(
        subparsers,
        "serve",
        "expose the atlas over MCP",
        "Serve the atlas as MCP tools, for agents that prefer tools to a shell. stdio by "
        "default, one process per developer, which is what rosio install registers. "
        "--http serves a team from one process and refuses a non loopback bind without a "
        "key. Starts without an atlas, and every tool says how to build one. Needs the mcp "
        "extra.",
        flags=("out", "quiet"),
    )
    serve.add_argument(
        "--atlas",
        metavar="FILE",
        help="atlas.json to answer from, a directory holding one, default is <out>/atlas.json",
    )
    transport = serve.add_mutually_exclusive_group()
    transport.add_argument(
        "--stdio", action="store_true", help="serve over stdio, the default transport"
    )
    transport.add_argument(
        "--http", action="store_true", help="serve streamable HTTP instead of stdio"
    )
    serve.add_argument(
        "--host", default="127.0.0.1", metavar="ADDR", help="http bind address, default 127.0.0.1"
    )
    serve.add_argument(
        "--port",
        type=int,
        default=SERVE_PORT,
        metavar="N",
        help=f"http port, default {SERVE_PORT}",
    )
    serve.add_argument(
        "--api-key",
        default="",
        metavar="KEY",
        help="require this key as Authorization: Bearer <key> or as an x-api-key header, "
        "required to bind anywhere but loopback. default: the ROSIO_API_KEY variable, "
        "which ps does not show",
    )
    serve.add_argument(
        "--path",
        default=SERVE_PATH,
        metavar="PATH",
        help=f"http path to serve on, default {SERVE_PATH}",
    )
    serve.add_argument(
        "--stateless",
        action="store_true",
        help="hold no session between requests, for a load balanced deployment",
    )
    serve.add_argument(
        "--workspace",
        default="",
        metavar="DIR",
        help="the directory migrate may read over http, default the directory above the "
        "rosio-out that holds the atlas, else the atlas's own directory",
    )

    hook = _sub(
        subparsers,
        "hook",
        "keep the atlas current from git itself",
        "Install the git hooks that rebuild the atlas after a commit, a checkout and a "
        "merge, register the merge driver that resolves two rebuilds of atlas.json without "
        "conflict markers, and report what a rebuild costs. A hook never blocks: a failed "
        "rebuild prints one line and exits zero.",
        flags=(),
    )
    hook.set_defaults(hook_command=None)
    hooks = hook.add_subparsers(dest="hook_command", metavar="action")

    hook_install = _hook_sub(
        hooks,
        "install",
        "write the hooks and register the merge driver",
        "Write post-commit, post-checkout and post-merge into this repository, with the "
        "interpreter embedded, because a hook started by a graphical client or by CI gets "
        "neither the shell profile nor the PATH the installer had. Registers the merge "
        "driver at the same time, so the two halves are never half configured.",
        flags=READ_FLAGS,
    )
    hook_install.add_argument("--repo", metavar="DIR", help="repository, default the working one")
    hook_install.add_argument(
        "--python", metavar="PATH", help="interpreter to embed, default the running one"
    )
    hook_install.add_argument(
        "--workspace",
        metavar="PATH",
        help="workspace the rebuild reads, relative to the repository",
    )
    hook_install.add_argument("--config", metavar="FILE", help="configuration the rebuild passes")
    hook_install.add_argument(
        "--force", action="store_true", help="replace a hook rosio did not write"
    )
    hook_install.add_argument("--dry-run", action="store_true", help="print what would be written")
    hook_install.add_argument(
        "--no-measure",
        dest="measure",
        action="store_false",
        help="skip the one timed rebuild the install runs to state its cost",
    )

    hook_status = _hook_sub(
        hooks,
        "status",
        "what is installed, what it embeds and what it costs",
        "Report each hook, the interpreter it embeds and whether that file still exists, "
        "the merge driver registration, and the wall time of the rebuilds recorded so far.",
    )
    hook_status.add_argument("--repo", metavar="DIR", help="repository, default the working one")

    hook_uninstall = _hook_sub(
        hooks,
        "uninstall",
        "remove the hooks and the merge driver registration",
        "Undo install. A hook file rosio did not write is left alone unless --force.",
    )
    hook_uninstall.add_argument("--repo", metavar="DIR", help="repository, default the working one")
    hook_uninstall.add_argument(
        "--force", action="store_true", help="delete a hook rosio did not write"
    )
    hook_uninstall.add_argument("--dry-run", action="store_true", help="print what would change")

    hook_run = _hook_sub(
        hooks,
        "run",
        "rebuild the atlas the way the hooks do",
        "Run the rebuild the hooks run, time it and record the time. Always exits zero.",
    )
    hook_run.add_argument("--repo", metavar="DIR", help="repository, default the working one")
    hook_run.add_argument("--event", default="manual", metavar="NAME", help="what triggered this")
    hook_run.add_argument(
        "--no-cache",
        dest="reuse",
        action="store_false",
        help="extract again even when nothing the extraction reads has moved",
    )

    hook_merge = _hook_sub(
        hooks,
        "merge",
        "the merge driver git calls for atlas.json",
        "Merge three versions of an atlas by union. A side that did not change a value never "
        "overrides the side that did, and where both sides changed one value differently "
        "both are kept, one in place and the other under x.merge in the file.",
    )
    hook_merge.add_argument("base", metavar="BASE", help="the common ancestor, git %%O")
    hook_merge.add_argument("ours", metavar="OURS", help="our version and the result, git %%A")
    hook_merge.add_argument("theirs", metavar="THEIRS", help="their version, git %%B")
    hook_merge.add_argument("--path", default="", metavar="PATH", help="the path being merged")
    hook_merge.add_argument("--result", metavar="FILE", help="write the merge here instead of OURS")

    _sub(
        subparsers,
        "version",
        "print the version and the atlas schema version",
        "Print the rosio version and the major.minor of the atlas schema it writes.",
        flags=("json",),
    )

    stamps = _sub(
        subparsers,
        "stamps",
        "check the header stamps of one recording",
        "Read the header stamp of every message in a ROS 1 or ROS 2 bag and check it against "
        "the time the recorder received it: backward steps, frozen and zero stamps, gaps, clock "
        "drift, offsets between topics and offsets that match a known clock fault such as the "
        "TAI or GPS leap offset. Needs no workspace and no ROS installation, only the stamps "
        "extra.",
        flags=("json",),
    )
    from rosio.stamps.cli import add_arguments as add_stamps_arguments

    add_stamps_arguments(stamps, json_flag=False)

    bundle = _sub(
        subparsers,
        "bundle",
        "hand a problem over with its evidence",
        "Create a portable archive of an atlas, reports and cited files, or open one.",
        flags=(),
    )
    actions = bundle.add_subparsers(dest="bundle_command", metavar="action")
    create = actions.add_parser(
        "create",
        help="write a portable bundle",
        description="Write the atlas, both reports, the files the findings cite, the "
        "configuration and the git revision into one archive.",
    )
    _add_global_flags(create, root=False, flags=("json", "quiet"))
    # Its own dest: the global --out names the output directory, which holds the atlas.
    create.add_argument(
        "--out",
        dest="archive",
        default=None,
        metavar="FILE",
        help="archive to write, default bundle.rosio.zip in the output directory",
    )
    create.add_argument(
        "workspace",
        nargs="?",
        default=".",
        metavar="PATH",
        help="workspace the atlas describes, default is the current directory",
    )
    create.add_argument(
        "--atlas",
        metavar="FILE",
        help="atlas.json to bundle, default rosio-out/atlas.json in the workspace",
    )
    create.add_argument(
        "--include",
        choices=["findings", "cited"],
        default="findings",
        help="findings takes the files the findings cite, cited every file any evidence "
        "row cites, default findings",
    )
    create.add_argument(
        "--note", default="", metavar="TEXT", help="a sentence for the person who opens it"
    )
    opened = actions.add_parser(
        "open",
        help="unpack a bundle and summarize the problem",
        description="Unpack a bundle into a directory that stands in for the sender's "
        "workspace and print the most severe findings with the file and line to edit.",
    )
    _add_global_flags(opened, root=False, flags=("json", "quiet"))
    opened.add_argument("file", metavar="FILE", help="the bundle archive")
    opened.add_argument(
        "--into",
        metavar="DIR",
        help="directory to unpack into, default beside the archive and named after it",
    )
    opened.add_argument("--force", action="store_true", help="overwrite existing files")

    move = _sub(
        subparsers,
        "migrate",
        "move a workspace to another ROS 2 release",
        "List every change a move between two releases asks of the workspace's own packages: "
        "dependencies the target does not release, headers it does not install, removed CMake "
        "functions and APIs, python modules gone from its interpreter. Each change cites its "
        "source. `apply` rewrites the changes that have an exact rewrite and writes the diff; "
        "the rest are listed with one sentence each. `verify` builds copied sources in the "
        "target container and matches errors to the plan.",
        flags=(),
    )
    steps = move.add_subparsers(dest="migrate_command", metavar="action")
    for action, summary in (
        ("plan", "list the changes, edit nothing"),
        ("apply", "rewrite the automatic changes and write the diff"),
        ("verify", "build copied sources in the target and compare errors with the plan"),
    ):
        step = steps.add_parser(action, help=summary, description=summary.capitalize() + ".")
        _add_global_flags(step, root=False, flags=READ_FLAGS)
        step.add_argument(
            "workspace",
            nargs="?",
            default=".",
            metavar="PATH",
            help="workspace whose source packages move, default is the current directory",
        )
        step.add_argument("--config", metavar="FILE", help="rosio.yaml, default the workspace's")
        step.add_argument(
            "--from",
            dest="start",
            metavar="RELEASE",
            help="the release the workspace builds against now, default $ROS_DISTRO",
        )
        step.add_argument(
            "--to",
            dest="target",
            required=True,
            metavar="RELEASE",
            help="the release to move to, for example jazzy",
        )
        if action == "verify":
            step.add_argument("--image", metavar="NAME", help="target container image")
        if action == "apply":
            step.add_argument(
                "--force", action="store_true", help="rewrite files with uncommitted changes"
            )

    return parser


GLOBAL_FLAGS = ("out", "json", "quiet", "deterministic")

READ_FLAGS = ("out", "json", "quiet")
"""What a verb that reads an atlas uses: where the atlas is, and how to print the answer."""


def _add_global_flags(
    parser: argparse.ArgumentParser, *, root: bool, flags: Sequence[str] = GLOBAL_FLAGS
) -> None:
    """The output flags, on the top level parser and on each subparser that uses them.

    A subparser writes its own defaults over the namespace the top level parser filled in,
    so only the top level copy carries defaults and the subcommand copy suppresses them.
    Otherwise a flag means one thing before the verb and another thing after it. Every
    default is None, so "not given" stays distinguishable from "given as false". A verb
    lists only the flags it reads, so its help never offers one that changes nothing.
    """
    unset = None if root else argparse.SUPPRESS
    if "out" in flags:
        parser.add_argument(
            "--out",
            default=unset,
            metavar="DIR",
            help="output directory, default rosio-out",
        )
    if "json" in flags:
        parser.add_argument(
            "--json", action="store_true", default=unset, help="print machine readable output"
        )
    if "quiet" in flags:
        parser.add_argument("--quiet", action="store_true", default=unset, help="print errors only")
    if "deterministic" in flags:
        parser.add_argument(
            "--deterministic",
            action="store_true",
            default=unset,
            help="keep output byte stable, drop timestamps and absolute paths (on by default)",
        )
        parser.add_argument(
            "--no-deterministic",
            dest="deterministic",
            action="store_false",
            default=argparse.SUPPRESS,
            help="allow timestamps and host specific values in the output",
        )


def _add_agent_flags(parser: argparse.ArgumentParser) -> None:
    """The flags install and uninstall share. The platform names are not a choices list,
    because that would import the platform table to build the parser for every verb."""
    parser.add_argument(
        "--scope",
        default="project",
        metavar="SCOPE",
        help="project or user, default project",
    )
    parser.add_argument("--root", metavar="DIR", help="project root, default the working directory")
    parser.add_argument("--home", metavar="DIR", help="home directory, used by the user scope")
    parser.add_argument("--dry-run", action="store_true", help="print what would change")


def _add_atlas_flags(parser: argparse.ArgumentParser) -> None:
    """The flags every verb that reads an atlas takes.

    `--atlas` names the file, and without it the file is looked for under `--out`, so the
    common case after an extract is the bare verb.
    """
    parser.add_argument(
        "--atlas",
        metavar="FILE",
        help="atlas.json to read, a directory holding one, default is <out>/atlas.json",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=12,
        metavar="N",
        help="rows per section, default 12",
    )


def _sub(
    subparsers: "argparse._SubParsersAction",
    name: str,
    summary: str,
    description: str,
    flags: Sequence[str] = GLOBAL_FLAGS,
) -> argparse.ArgumentParser:
    parser = subparsers.add_parser(
        name,
        help=summary,
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _add_global_flags(parser, root=False, flags=flags)
    parser.set_defaults(command=name)
    return parser


def _hook_sub(
    subparsers: "argparse._SubParsersAction",
    name: str,
    summary: str,
    description: str,
    flags: Sequence[str] = ("json", "quiet"),
) -> argparse.ArgumentParser:
    """One action under `hook`. The output flags are repeated so --json works after it."""
    parser = subparsers.add_parser(
        name,
        help=summary,
        description=description,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    _add_global_flags(parser, root=False, flags=flags)
    parser.set_defaults(hook_command=name)
    return parser


def cmd_init(args: argparse.Namespace) -> int:
    from pathlib import Path

    from rosio import scaffold
    from rosio.config import CONFIG_NAME, default_config

    workspace = Path(args.workspace).expanduser()
    if not workspace.is_dir():
        raise UsageError(f"workspace directory does not exist: {workspace}")
    packages = args.package or ()
    target = Path(args.config).expanduser().absolute() if args.config else None
    if args.print:
        base = target.parent if target else workspace
        print(scaffold.render(default_config(str(workspace), packages), base), end="")
        return EXIT_OK
    wanted = target or workspace / CONFIG_NAME
    target, cfg = scaffold.write(workspace, force=args.force, packages=packages, target=target)
    if args.quiet:
        return EXIT_OK
    if target != wanted:
        print(f"{PROG} init: {wanted} is not writable, so the file went here instead.")
    command = f"{PROG} extract {args.workspace}"
    if target.parent.absolute() != workspace.absolute():
        here = target.parent.absolute() == Path.cwd()
        command += f" --config {target.name if here else target}"
    print(
        f"{PROG} init: wrote {target} with {plural(len(cfg.roots), 'root')} and "
        f"{plural(len(cfg.profiles), 'profile')}. Edit the profiles, then run: {command}"
    )
    if not cfg.profiles:
        runs = scaffold.run_entrypoints(cfg)
        print(
            f"{PROG} init: no launch file found, so no profile was written. A profile can start "
            "executables instead, one entrypoint line each"
            + (": " + ", ".join(runs[:5]) + (", ..." if len(runs) > 5 else "") if runs else "")
            + ". See docs/configuration.md."
        )
    if len(cfg.profiles) > scaffold.MANY_PROFILES and not packages:
        top = scaffold.top_level(cfg)
        print(
            f"{PROG} init: that is a profile per launch file. Keep the ones a robot starts "
            f"with, or write one package's with --package NAME. {plural(len(top), 'file')} "
            "no other launch file includes: "
            + ", ".join(top[:8])
            + (", ..." if len(top) > 8 else "")
        )
    return EXIT_OK


def cmd_stamps(args: argparse.Namespace) -> int:
    if args.bag is None:
        print(f"{PROG} stamps: a bag path is required", file=sys.stderr)
        return EXIT_USAGE
    from rosio.stamps.cli import run

    return run(args)


def cmd_version(args: argparse.Namespace) -> int:
    from rosio.model import SCHEMA_VERSION, dumps

    if args.json:
        print(dumps({"version": __version__, "atlas_schema_version": SCHEMA_VERSION}), end="")
    else:
        print(f"{PROG} {__version__}, atlas schema {SCHEMA_VERSION}")
    return EXIT_OK


def cmd_extract(args: argparse.Namespace) -> int:
    """Read a workspace and write atlas.json.

    The wall time is measured around the extraction and printed with the summary. It is
    deliberately not inside the atlas: two runs over unchanged inputs must be byte identical.
    """
    import time
    from pathlib import Path

    from rosio import extract as extract_mod
    from rosio import model
    from rosio.config import DEFAULT_OUT_DIR, load_config, locate_config, with_install_tree
    from rosio.errors import EXIT_CONTRACT, CollectorError
    from rosio.workspace import Workspace

    bags = _bag_paths(args)
    if bags and args.no_runtime:
        raise UsageError("--no-runtime and --bag ask for opposite things, pass one of them")
    layers = sorted(set(args.layer or ["coded", "declared"]))

    workspace = Path(args.workspace).expanduser()
    if not workspace.is_dir():
        raise UsageError(f"workspace directory does not exist: {workspace}")

    say = _progress("extract", quiet=args.quiet)
    source = locate_config(
        args.config,
        workspace=str(workspace),
        note=lambda text: print(f"{PROG} extract: {text}", file=sys.stderr),
    )
    if not args.quiet:
        print(
            f"{PROG} extract: config {source.absolute()}"
            if source
            else f"{PROG} extract: no rosio.yaml found, one profile per launch file "
            f"({PROG} init writes that out to edit)",
            file=sys.stderr,
        )
    cfg = load_config(str(source) if source else None, workspace=str(workspace))
    cfg = _with_overrides(cfg, args, DEFAULT_OUT_DIR)
    if args.install_tree:
        cfg = with_install_tree(cfg, args.install_tree)
    ws = Workspace.discover(_source_roots(cfg), cfg.ignore)
    say(f"indexed {len(ws.files())} files")
    if not bags and not _bag_roots(cfg) and not _holds_ros(ws):
        raise CollectorError(
            f"no package.xml and no launch file under {workspace.absolute()}, so there is "
            f"nothing to extract. is this the workspace? run {PROG} extract <workspace>"
        )

    collectors = _collectors(args, extract_mod)
    if collectors is None and layers != sorted({"coded", "declared"}):
        collectors = [name for layer in layers for name in extract_mod.default_collectors(layer)]
    instances = {}
    if not args.no_runtime and (bags or _bag_roots(cfg)):
        collectors = list(collectors or extract_mod.default_collectors())
        collectors.extend(extract_mod.collectors_for(model.Layer.RECORDED.value))
        if bags:
            from rosio.collect.bag import BagCollector
            from rosio.collect.stamps import StampsCollector

            instances["bag"] = BagCollector(paths=tuple(bags))
            instances["stamps"] = StampsCollector(paths=tuple(bags))
    started = time.perf_counter()
    atlas = extract_mod.run(cfg, ws, collectors=collectors, instances=instances, progress=say)
    elapsed = time.perf_counter() - started

    report = extract_mod.summary(atlas)
    target = extract_mod.out_path(cfg)
    if args.json:
        payload = dict(report)
        payload["out"] = str(target)
        payload["bytes"] = target.stat().st_size
        payload["volatile"] = {"wall_seconds": round(elapsed, 3)}
        print(model.dumps(payload), end="")
    elif not args.quiet:
        print(f"{PROG} extract: {target.absolute()} ({_bytes(target)})")
        _print_summary(report, elapsed, atlas)
        _stale_note(target, atlas)

    if args.fail_on_violation and report["exceptions"]:
        print(
            f"{PROG}: {plural(len(report['exceptions']), 'problem')} recorded during extraction",
            file=sys.stderr,
        )
        return EXIT_CONTRACT
    return EXIT_OK


def _progress(verb: str, *, quiet: bool):
    """One stderr line per finished phase, with its seconds and the seconds so far.

    A step of a phase that repeats per profile (`step=True`) rewrites one line in a
    terminal, and anywhere else stays quiet until the phase ends, which prints its last
    step once with the time the whole phase took. Sixty one profiles were sixty one lines."""
    import time

    started = last = time.perf_counter()
    terminal = sys.stderr.isatty()
    phase_started: float | None = None
    phase_label = ""

    def line(label: str, since: float, now: float) -> str:
        return f"{PROG} {verb}: {label}, {now - since:.1f}s ({now - started:.1f}s elapsed)"

    def say(label: str, *, step: bool = False) -> None:
        nonlocal last, phase_started, phase_label
        now = time.perf_counter()
        if quiet:
            last = now
            return
        if step:
            phase_started = last if phase_started is None else phase_started
            phase_label = label
            if terminal:
                text = f"\r{line(label, phase_started, now)}\x1b[K"
                print(text, end="", file=sys.stderr, flush=True)
            last = now
            return
        if phase_started is not None:
            # On a terminal the last step already stands on the line with these numbers, so
            # the line is ended rather than written a second time.
            done = line(phase_label, phase_started, last)
            print("" if terminal else done, file=sys.stderr)
            phase_started = None
        print(line(label, last, now), file=sys.stderr, flush=True)
        last = now

    return say


def _holds_ros(ws) -> bool:
    """Whether a tree holds anything a collector reads first: a package or a launch file. An
    atlas of nothing written with exit zero read as a clean workspace."""
    from rosio.workspace import LAUNCH_SUFFIXES

    return bool(ws.packages) or any(name.endswith(LAUNCH_SUFFIXES) for name in ws.files())


def _with_overrides(cfg, args: argparse.Namespace, default_out: str):
    """Command line over config file over default, for the three settings both can carry.

    An unset flag is None, so --no-deterministic turns determinism off from either side of
    the verb instead of losing to a configured true. Determinism itself is on unless the
    command line says otherwise: config.deterministic defaults to false and cannot tell an
    absent key from a written false, so reading it here would turn the default output non
    deterministic for every config that stays silent.
    """
    import dataclasses

    out_dir = args.out or cfg.out_dir or default_out
    profiles = cfg.profiles
    if args.profile:
        wanted = list(dict.fromkeys(args.profile))
        known = {spec.slug for spec in cfg.profiles}
        missing = [slug for slug in wanted if slug not in known]
        if missing:
            raise UsageError(
                f"unknown profile: {', '.join(missing)}. "
                f"this config declares: {', '.join(sorted(known)) or 'none'}"
            )
        profiles = tuple(spec for spec in cfg.profiles if spec.slug in set(wanted))
    return dataclasses.replace(
        cfg,
        out_dir=out_dir,
        profiles=profiles,
        launch_sandbox=getattr(args, "launch_sandbox", None) or cfg.launch_sandbox,
        deterministic=(
            DEFAULT_DETERMINISTIC if args.deterministic is None else bool(args.deterministic)
        ),
    )


def _bag_paths(args: argparse.Namespace) -> list[str]:
    """The recordings named on the command line, each one checked before anything runs."""
    from pathlib import Path

    out: list[str] = []
    for raw in dict.fromkeys(getattr(args, "bag", None) or ()):
        path = Path(raw).expanduser()
        if not path.is_dir():
            raise UsageError(f"bag directory does not exist: {path}")
        out.append(str(path))
    return out


def _bag_roots(cfg) -> list[str]:
    from rosio.model import Root

    return [spec.path for spec in cfg.roots if spec.kind == Root.BAG.value]


def _source_roots(cfg):
    """The roots the workspace indexes. A recording is not a source tree: indexing one would
    put its storage files into the file table and hash a gigabyte to no end, and the bag
    collector reads it through its own root anyway."""
    from rosio.model import Root

    return tuple(spec for spec in cfg.roots if spec.kind != Root.BAG.value)


def _collectors(args: argparse.Namespace, extract_mod) -> list[str] | None:
    """The names this extract run may call, refused before anything is read.

    A collector that attaches to a live middleware is refused here as well as in `extract`,
    so the answer arrives before a workspace is walked and never depends on which layer the
    selection happened to reach first.
    """
    if not args.collector:
        return None
    known = extract_mod.available()
    wanted = list(dict.fromkeys(args.collector))
    missing = [name for name in wanted if name not in known]
    if missing:
        raise UsageError(f"unknown collector: {', '.join(missing)}. registered: {', '.join(known)}")
    live = sorted(name for name in wanted if name in extract_mod.OPT_IN)
    if live:
        raise UsageError(
            f"{', '.join(live)} attaches to a live system and cannot be selected here. "
            f"run {PROG} probe, which takes the domain id to attach to"
        )
    return wanted


LAYER_DOWN = "LaunchUnavailable"
"""Exception type `collect/launch_declared` writes when the launch machinery could not be
imported at all. It is printed before the counts, because the counts under it are a lower
bound over files that were never loaded and read as a complete atlas otherwise."""


def _print_summary(report: dict, elapsed: float, atlas=None) -> None:
    """What the atlas holds after a run. The caller prints the first line, because the verb
    that wrote the file is the caller's to name."""
    from rosio.query import readable

    for row in report["exceptions"]:
        if str(row.get("exc_type", "")) != LAYER_DOWN:
            continue
        print(f"  the launch layer did not happen: {readable(str(row.get('message', '')), atlas)}")
        print("    every launch entry point is marked unloadable, and the node,")
        print("    parameter and plugin counts below are a lower bound")
        break
    profiles = report["profiles"]
    shown = ", ".join(profiles[:PROFILES_SHOWN]) or "none"
    if len(profiles) > PROFILES_SHOWN:
        shown += f" and {len(profiles) - PROFILES_SHOWN} more, rosio extract --json lists all"
    print(f"  profiles {len(profiles)}: {shown}")
    print(f"  layers: {', '.join(report['layers']) or 'none'}")
    print(f"  entities {report['entities']}: {_counts(report['entities_by_kind'])}")
    print(f"  edges {report['edges']}: {_counts(report['edges_by_kind'])}")
    print(f"  unresolved {report['unresolved']}: {_counts(report['unresolved_by_want'])}")
    print(f"  resolution rate {report['resolution_rate']:.3f} over {report['resolved']} resolved")
    findings = report["findings"]
    print("  findings: " + ", ".join(f"{count} {name}" for name, count in sorted(findings.items())))
    top = sorted(report["findings_by_rule"].items(), key=lambda item: (-item[1], item[0]))[:5]
    if top:
        print("    most: " + ", ".join(f"{name} {count}" for name, count in top))
        print(f"    {PROG} drift prints each one with the line to edit")
    if report["rules_failed"]:
        print(f"    rules that could not be trusted: {', '.join(report['rules_failed'])}")
    problems = report["exceptions"]
    if problems:
        print(f"  problems recorded {len(problems)}:")
        shown = _problem_lines(problems, atlas)
        for text, count in shown[:5]:
            print(f"    {text}" + (f", recorded {count} times" if count > 1 else ""))
        if len(shown) > 5:
            hidden = sum(count for _, count in shown[5:])
            print(f"    and {hidden} more, {PROG} extract --json lists every one under exceptions")
    print(f"  wall time {elapsed:.1f}s, measured outside the atlas")


OPAQUE = r"OpaqueFunction\(fn[0-9a-f]+\)"
"""How the launch walk names a function it could not name, by a digest of its code."""


def _problem_lines(problems: list, atlas=None) -> list[tuple[str, int]]:
    """One line per distinct problem with how often it was recorded, in the order recorded.
    A launch file walked by three profiles raised three identical rows, and the digest a walk
    gives an unnamed function or the token for a process output is no name to a reader."""
    import re

    from rosio import here
    from rosio.query import readable

    counted: dict[str, int] = {}
    for row in problems:
        where = str(row.get("path") or row.get("file") or "")
        if where.startswith("@collector/"):
            where = f"the {where.split('/', 1)[1]} collector"
        elif where == "@launch":
            where = "the launch walk"
        if row.get("exc_type") == "ContractError":
            # A broken invariant inside rosio, not a property of the file it was reading.
            text = f"{where}: rosio hit an internal error here, please report it: "
            text += str(row.get("message", ""))
        elif row.get("exc_type") == "CollectorError":
            # A collector's own refusal, said once: no class name, no collector prefix and
            # no second copy of the path the line starts with.
            message = re.sub(r"^[a-z_]+\.[a-z_]+: ", "", str(row.get("message", "")))
            message = message.replace(f" {where}: ", " it: ") if where else message
            text = f"{where}: {message}".strip(": ")
        else:
            text = f"{where} {row.get('exc_type', '')}: {row.get('message', '')}".strip()
        text = re.sub(OPAQUE, "an OpaqueFunction", text)
        text = here(readable(text.replace("@output", "<its output file>"), atlas))
        counted[text] = counted.get(text, 0) + 1
    return list(counted.items())


def _stale_note(target, atlas) -> None:
    """Name the rendered files beside the atlas that describe an earlier one."""
    from rosio.render import stale

    old = stale(target.parent, atlas)
    if old:
        verb = "describes" if len(old) == 1 else "describe"
        print(f"  {', '.join(old)} {verb} an earlier atlas, {PROG} render brings it up to date")


def _counts(by_key: dict) -> str:
    rows = sorted(by_key.items(), key=lambda item: (-item[1], item[0]))
    return ", ".join(f"{name} {count}" for name, count in rows) or "none"


def _bytes(target) -> str:
    size = target.stat().st_size
    if size < 1024:
        return f"{size} B"
    if size < 1024 * 1024:
        return f"{size / 1024:.1f} KiB"
    return f"{size / (1024 * 1024):.1f} MiB"


# ---------------------------------------------------------------------------
# the verbs that read an atlas off disk


def _atlas_path(args: argparse.Namespace, path: str | None = None):
    """Where an atlas is read from: what was named, atlas.json under the output dir, or the
    atlas of the unpacked bundle this directory is."""
    from pathlib import Path

    from rosio.config import DEFAULT_OUT_DIR
    from rosio.extract import ATLAS_NAME

    named = path or getattr(args, "atlas", None)
    target = Path(named).expanduser() if named else Path(args.out or DEFAULT_OUT_DIR) / ATLAS_NAME
    unnamed = not named and not getattr(args, "out", None)
    if unnamed and not target.exists() and Path("manifest.json").is_file():
        target = Path(ATLAS_NAME)
    return target / ATLAS_NAME if target.is_dir() else target


def load_atlas(args: argparse.Namespace, path: str | None = None):
    """The atlas one verb answers from. Nothing here extracts."""
    from rosio.model import Atlas

    target = _atlas_path(args, path)
    if not target.is_file():
        raise UsageError(
            f"no atlas at {target}. run {PROG} extract first, or name one with --atlas"
        )
    return Atlas.loads(target.read_text(encoding="utf-8"))


_SEVERITY_ANSI = {"error": "1;31", "warn": "33", "info": "2"}


def _paint(text: str, stream=None) -> str:
    """Severity words at the head of a line in their terminal colour, on a terminal only.
    NO_COLOR turns it off and FORCE_COLOR on, the conventions most tools share."""
    import os
    import re

    stream = stream or sys.stdout
    if os.environ.get("NO_COLOR") or os.environ.get("TERM") == "dumb":
        return text
    if not os.environ.get("FORCE_COLOR") and not stream.isatty():
        return text
    return re.sub(
        r"^(\s*)(error|warn|info)(?=[ :])",
        lambda m: f"{m[1]}[{_SEVERITY_ANSI[m[2]]}m{m[2]}[0m",
        text,
        flags=re.M,
    )


def _answer(args: argparse.Namespace, answer) -> int:
    from rosio.model import dumps

    if args.json:
        print(dumps(answer.to_json()), end="")
    elif not args.quiet:
        print(_paint(answer.text()))
    return EXIT_OK if answer.ok else EXIT_USAGE


def _words(values) -> str:
    if isinstance(values, str):
        return values.strip()
    return " ".join(str(item) for item in values or ()).strip()


def cmd_drift(args: argparse.Namespace) -> int:
    """The findings the atlas carries, filtered for printing and graded for the gate.

    The print threshold and the gate severity are two decisions: the gate reads the findings
    the run holds before `--min-severity` took anything out of them, so a build told to
    print errors and to fail on warnings fails on a warning it never printed.

    A waiver is a third decision and stays separate from both. A waived finding is printed
    like any other, with the reason beside it, and only the gate stops reading it, so a team
    that has accepted one known finding keeps the gate on instead of turning it off.
    """
    from rosio import query
    from rosio.drift import engine, rules
    from rosio.errors import EXIT_DRIFT

    atlas = load_atlas(args)
    if DRIFT_CAPABILITY not in atlas.capabilities:
        raise UsageError(
            "this atlas was written before the rules ran over it, so it carries no findings "
            f"and this verb would report a clean stack. it declares {atlas.capabilities}, "
            f"and {DRIFT_CAPABILITY} is what says the rules ran, so extract it again"
        )
    wanted = list(dict.fromkeys(args.only or []))
    unknown = [name for name in wanted if name not in rules.COMMITTED]
    if unknown:
        raise UsageError(
            f"unknown drift rule: {', '.join(unknown)}. "
            "a mistyped rule id would report no drift, which reads like a clean stack"
        )
    if wanted:
        atlas.findings = [item for item in atlas.findings if item.rule in set(wanted)]
    waivers = _waivers(args)
    graded, matched = engine.waive(atlas.findings, [item.key for item in waivers])
    answer = query.drift(atlas, min_severity=args.min_severity, limit=args.limit, waived=matched)
    _report_waivers(answer, waivers, matched, args.limit)
    # A rule that raised while the atlas was built left nothing behind it but its name.
    failed = [
        name
        for name in (atlas.counters.get("drift") or {}).get("failed") or []
        if not wanted or name in wanted or name not in rules.COMMITTED
    ]
    if failed:
        answer.data["rules_failed"] = failed
        answer.lines.append(
            f"  rules that could not be trusted, so their findings are missing: {', '.join(failed)}"
        )
    code = _answer(args, answer)
    failures = [
        engine.RuleFailure(rule=name, kind="raised", exc_type="", message="") for name in failed
    ]
    if engine.crosses_gate(engine.DriftRun(gated=graded, failures=failures), args.fail_on):
        crossed = [item for item in graded if _at_or_above(item.severity, args.fail_on)]
        rules_hit = sorted({item.rule for item in crossed})
        if crossed:
            print(
                f"{PROG}: {plural(len(crossed), 'finding')} at "
                f"{args.fail_on} or above ({', '.join(rules_hit)}), the gate is "
                f"--fail-on {args.fail_on}",
                file=sys.stderr,
            )
        if failed:
            print(
                f"{PROG}: {plural(len(failed), 'rule')} could not be trusted "
                f"({', '.join(failed)}), which the gate grades as error, extract again and "
                "report the rule if it fails again",
                file=sys.stderr,
            )
        return EXIT_DRIFT
    return code


def _waivers(args: argparse.Namespace):
    """The findings this repository has accepted: the flag, or the configuration key.

    The command line wins whole rather than adding to the configuration, so a run told to
    grade against one file is graded against that file and nothing else.
    """
    from rosio.config import load_waivers, waiver_files

    named = list(dict.fromkeys(args.waivers or []))
    return load_waivers(named or list(waiver_files(args.config)))


def _report_waivers(answer, waivers, matched: dict, limit: int) -> None:
    """Say what a waiver did, on the answer that prints the findings it acted on.

    A waiver key that matched nothing is named too. A stored key rots when the finding it
    covers is fixed or its subject is renamed, and a gate that never mentions it keeps a
    silence nobody asked for.
    """
    if not waivers:
        return
    reasons = {item.key: item.reason for item in waivers}
    rows = [
        {"finding": finding, "key": key, "reason": reasons[key]}
        for finding, key in sorted(matched.items())
    ]
    stale = sorted(set(reasons) - set(matched.values()))
    answer.data["waived"] = rows
    answer.data["waivers_matching_nothing"] = stale
    answer.lines.append(
        f"  {len(rows)} of {len(reasons)} waivers matched, "
        "printed above and not graded by --fail-on"
    )
    for row in rows[:limit]:
        answer.lines.append(f"        waived {row['finding']}")
        answer.lines.append(f"          {row['reason']}")
    for key in stale[:limit]:
        answer.lines.append(f"        waiver matched nothing, delete it: {key}")


def _at_or_above(severity: str, floor: str) -> bool:
    from rosio.model import SEVERITY_ORDER

    limit = SEVERITY_ORDER.get(floor)
    if limit is None:
        return False
    return SEVERITY_ORDER.get(severity, len(SEVERITY_ORDER)) <= limit


def cmd_explain(args: argparse.Namespace) -> int:
    from rosio import query

    selector = _words(args.selector)
    if not selector:
        raise UsageError(
            "explain needs a finding id, a fingerprint or an entity, for example /planner_server"
        )
    return _answer(args, query.explain(load_atlas(args), selector, limit=args.limit))


def cmd_param(args: argparse.Namespace) -> int:
    from rosio import query

    selector = _words(args.selector)
    if not selector:
        raise UsageError(
            "param needs a node and a parameter name, for example "
            "/planner_server GridBased.cost_penalty"
        )
    # Split here rather than handing over one string: the unnamed half reaches the not found
    # sentence as the word None.
    node, _sep, name = selector.partition(" ")
    answer = query.param(load_atlas(args), node, name.strip() or None, profile=args.profile)
    return _answer(args, answer)


def cmd_query(args: argparse.Namespace) -> int:
    from rosio import query

    question = _words(args.question)
    if not question:
        raise UsageError('query needs a question, for example "which nodes publish cmd_vel"')
    return _answer(args, query.query(load_atlas(args), question, limit=args.limit))


def cmd_path(args: argparse.Namespace) -> int:
    from rosio import query

    if not args.source or not args.target:
        raise UsageError("path needs both ends, for example path /ouster/os_driver /cmd_vel")
    answer = query.path(load_atlas(args), args.source, args.target, max_hops=max(1, args.max_hops))
    return _answer(args, answer)


def cmd_diff(args: argparse.Namespace) -> int:
    from rosio import query

    profiles = list(args.profile or [])
    revisions = list(args.rev or [])
    if revisions:
        return _diff_revisions(args, revisions, profiles)
    if profiles and len(profiles) != 2:
        raise UsageError(
            f"diff needs exactly two --profile values to compare profiles, got {len(profiles)}"
        )
    if not profiles and not (args.before and args.after):
        raise UsageError(
            "diff needs two atlases, or two --profile values of one atlas, for example "
            "diff before/atlas.json after/atlas.json"
        )
    left = load_atlas(args, args.before)
    right = load_atlas(args, args.after) if args.after else None
    answer = query.diff(
        left,
        right,
        left_profile=profiles[0] if profiles else None,
        right_profile=profiles[1] if profiles else None,
        limit=args.limit,
    )
    return _answer(args, answer)


def _diff_revisions(args: argparse.Namespace, revisions: list[str], profiles: list[str]) -> int:
    """Build two revisions of one workspace and compare them. The question after a rebase."""
    from rosio import history

    if len(revisions) != 2:
        raise UsageError(f"diff needs exactly two --rev values, got {len(revisions)}")
    if profiles:
        raise UsageError(
            "--rev and --profile ask two different questions, run one of them at a time"
        )
    answer = history.compare_revisions(
        revisions[0],
        revisions[1],
        repo=args.repo,
        workspace=args.workspace,
        limit=args.limit,
        cache_rules=args.cache_rules,
    )
    return _answer(args, answer)


def cmd_unresolved(args: argparse.Namespace) -> int:
    from rosio import query

    atlas = load_atlas(args)
    wants = list(dict.fromkeys(args.want or []))
    if wants:
        atlas.unresolved = [entry for entry in atlas.unresolved if entry.want in set(wants)]
    answer = query.unresolved(atlas, want=wants[0] if len(wants) == 1 else None, limit=args.limit)
    return _answer(args, answer)


def cmd_graph(args: argparse.Namespace) -> int:
    """The MCP graph tool on the command line, and the same selection drawn as a diagram."""
    from rosio.serve import tools

    if args.json:
        raise UsageError("graph prints text, dot or mermaid, choose with --format")
    atlas = load_atlas(args)
    chosen = {
        "profile": args.profile,
        "focus": args.focus,
        "depth": args.depth,
        "wrappers": args.wrappers,
    }
    if args.format == "text":
        text = tools.graph(atlas, **chosen, limit=args.limit, cli=True) + "\n"
        failed = bool(args.focus) and bool(tools.select_graph(atlas, **chosen).complaint)
    else:
        text = tools.diagram(atlas, args.format, **chosen, limit=args.limit)
        failed = False
    if not args.quiet:
        print(text, end="")
    return EXIT_USAGE if failed else EXIT_OK


def cmd_render(args: argparse.Namespace) -> int:
    """Write the human readable outputs from an atlas that is already on disk.

    The outputs land beside the atlas they were rendered from unless `--out` says otherwise,
    so `extract` then `render` fills one directory and a report can never describe an atlas
    from a different run.
    """
    from rosio.render import write_all

    atlas = load_atlas(args)
    written = write_all(
        atlas,
        out_dir=args.out or _atlas_path(args, args.atlas).parent,
        formats=args.format or ["md", "html"],
    )
    if not written:
        raise UsageError(f"no renderer wrote anything for {', '.join(args.format or [])}")
    if args.json:
        from rosio.model import dumps

        print(dumps({"verb": "render", "ok": True, "data": {"written": sorted(written)}}), end="")
    elif not args.quiet:
        for target in written:
            print(f"{PROG} render: {target}")
    return EXIT_OK


# ---------------------------------------------------------------------------
# the live system


def cmd_probe(args: argparse.Namespace) -> int:
    """Add the observed layer to an atlas, without extracting anything again.

    A probe that reached nothing exits with the collector code. The atlas is still written,
    because the run and the exception it recorded are the evidence of what happened.
    """
    import time

    from rosio import extract as extract_mod
    from rosio import model
    from rosio.collect import probe as probe_collect
    from rosio.errors import EXIT_COLLECTOR
    from rosio.workspace import Workspace

    domain = _pin_domain(args)
    source = _atlas_path(args, args.atlas)
    atlas = _existing_atlas(source)
    cfg = _probe_config(args, source)
    ws = Workspace.discover(_source_roots(cfg), cfg.ignore)

    settle = probe_collect.DEFAULT_SETTLE if args.settle is None else max(0.0, args.settle)
    instances = {
        "probe": probe_collect.ProbeCollector(
            window=max(0.0, args.duration), settle=settle, rmws=args.rmw, stamps=args.stamps
        )
    }
    started = time.perf_counter()
    written = extract_mod.run(
        cfg,
        ws,
        collectors=extract_mod.collectors_for(model.Layer.OBSERVED.value),
        instances=instances,
        atlas=atlas,
        opt_in=sorted(extract_mod.OPT_IN),
    )
    elapsed = time.perf_counter() - started

    status = written.layers[model.Layer.OBSERVED.value]
    report = extract_mod.summary(written)
    target = extract_mod.out_path(cfg)
    seen = _observed_counts(written)
    if args.json:
        payload = dict(report)
        payload["out"] = str(target)
        payload["bytes"] = target.stat().st_size
        payload["observed"] = {"present": status.present, "reason": status.reason, **seen}
        payload["volatile"] = {"wall_seconds": round(elapsed, 3)}
        print(model.dumps(payload), end="")
    elif not args.quiet:
        print(f"{PROG} probe: {target} ({_bytes(target)})")
        print(f"  domain {domain}, window {args.duration:g}s, settle {settle:g}s")
        print(f"  observed: {_counts(seen)}")
        _print_summary(report, elapsed, written)
        _stale_note(target, written)
    if not status.present:
        print(
            f"{PROG}: the observed layer is empty ({status.reason or 'nothing was reported'})",
            file=sys.stderr,
        )
        return EXIT_COLLECTOR
    return EXIT_OK


def _pin_domain(args: argparse.Namespace) -> str:
    """The domain the probe attaches to, from the flag or from what the caller exported.

    Refusing here rather than in the collector is deliberate: by the time a child is running
    it has already joined a domain, and on a machine attached to a vehicle the wrong one is
    the worst thing this tool can do.
    """
    import os

    given = str(args.domain_id or "").strip()
    domain = given or os.environ.get(DOMAIN_ENV, "").strip()
    if not domain:
        raise UsageError(
            f"probe needs a domain: pass --domain-id N or export {DOMAIN_ENV} first. "
            "this machine can be attached to a vehicle, so the domain is never guessed"
        )
    if not domain.isdigit():
        raise UsageError(f"a domain id is a number, got {domain!r}")
    os.environ[DOMAIN_ENV] = domain
    if args.localhost_only:
        os.environ[LOCALHOST_ENV] = "1"
    return domain


def _existing_atlas(source):
    """The atlas the probe extends, or None when there is none to extend."""
    from rosio.extract import carries_layer
    from rosio.model import Atlas, Layer

    if not source.is_file():
        return None
    atlas = Atlas.loads(source.read_text(encoding="utf-8"))
    if carries_layer(atlas, Layer.OBSERVED.value):
        raise UsageError(
            f"{source} already carries the observed layer, and a fact slot is written by one "
            "run. probe a fresh atlas instead: name it with --atlas and write the result "
            "somewhere else with --out"
        )
    return atlas


def _probe_config(args: argparse.Namespace, source):
    """The configuration one probe runs under: the workspace it describes, the setup files
    the child sources, and the profile the observation is filed against."""
    import dataclasses
    from pathlib import Path

    from rosio.config import DEFAULT_OUT_DIR, load_config

    workspace = Path(args.workspace).expanduser()
    if not workspace.is_dir():
        raise UsageError(f"workspace directory does not exist: {workspace}")
    cfg = load_config(args.config, workspace=str(workspace))
    profiles = cfg.profiles
    if args.profile:
        known = {spec.slug for spec in cfg.profiles}
        if args.profile not in known:
            raise UsageError(
                f"unknown profile: {args.profile}. "
                f"this config declares: {', '.join(sorted(known)) or 'none'}"
            )
        profiles = tuple(spec for spec in cfg.profiles if spec.slug == args.profile)
    out_dir = args.out or (str(source.parent) if args.atlas else "") or cfg.out_dir
    return dataclasses.replace(
        cfg,
        out_dir=out_dir or DEFAULT_OUT_DIR,
        profiles=profiles,
        ros_setup=tuple(args.ros_setup) if args.ros_setup else cfg.ros_setup,
        deterministic=(
            DEFAULT_DETERMINISTIC if args.deterministic is None else bool(args.deterministic)
        ),
    )


def _observed_counts(atlas) -> dict:
    """What the observed layer put into the atlas, counted from the facts themselves."""
    from rosio.model import EdgeKind, Layer, NodeKind

    layer = Layer.OBSERVED.value
    wanted = {
        NodeKind.NODE.value: "nodes",
        NodeKind.PARAM.value: "parameters",
        NodeKind.FRAME.value: "frames",
        NodeKind.TOPIC.value: "topics",
    }
    counts = dict.fromkeys(sorted(wanted.values()), 0)
    for entity in atlas.entities.values():
        name = wanted.get(entity.kind)
        if name and any(fact.layer == layer for fact in entity.facts):
            counts[name] += 1
    seen = [edge for edge in atlas.edges.values() if any(f.layer == layer for f in edge.facts)]
    # The probe states a frame by the transform that names it, not by a fact on the frame.
    counts["frames"] = len(
        {
            end
            for edge in seen
            if edge.kind == EdgeKind.TF_PARENT.value
            for end in (edge.src, edge.dst)
        }
    )
    endpoint_kinds = {
        EdgeKind.PUBLISHES.value,
        EdgeKind.SUBSCRIBES.value,
        EdgeKind.SERVES.value,
        EdgeKind.CALLS.value,
    }
    counts["endpoints"] = sum(1 for edge in seen if edge.kind in endpoint_kinds)
    return counts


# ---------------------------------------------------------------------------
# the agent surface


def cmd_install(args: argparse.Namespace) -> int:
    """Write the files one assistant reads, so it can call these subcommands."""
    from rosio.agent import install as agent

    if args.list or not args.target:
        return agent.main(["--list"] if args.list else [])
    report = agent.install(
        args.target,
        args.scope,
        root=args.root,
        home=args.home,
        dry_run=args.dry_run,
        force=args.force,
    )
    print(report.text())
    return EXIT_OK


def cmd_uninstall(args: argparse.Namespace) -> int:
    """Remove what install wrote, and nothing else."""
    from rosio.agent import install as agent

    if not args.target:
        raise UsageError(
            "name the assistant to remove rosio from, for example: rosio uninstall codex"
        )
    report = agent.uninstall(
        args.target,
        args.scope,
        root=args.root,
        home=args.home,
        dry_run=args.dry_run,
        force=args.force,
    )
    print(report.text())
    return EXIT_OK


def _require_mcp() -> None:
    """Refuse before a transport opens, naming the command that fixes it. The check is the
    server's, read inside the function so building the parser imports nothing from it."""
    from rosio.serve.server import mcp_problem

    problem = mcp_problem()
    if problem:
        raise UsageError(problem)


def cmd_serve(args: argparse.Namespace) -> int:
    """Serve one atlas as MCP tools until the caller stops it.

    Nothing is written to stdout: the stdio transport owns it, and a stray line there is a
    protocol error rather than a log message.
    """
    _require_mcp()
    from rosio.serve.server import Options, run

    options = Options(
        atlas=str(_atlas_path(args, args.atlas)),
        http=args.http,
        host=args.host,
        port=args.port,
        api_key=args.api_key,
        path=args.path,
        stateless=args.stateless,
        workspace=args.workspace,
    ).check()
    if options.http and not args.quiet:
        guard = "key required" if options.api_key else "loopback only, no key"
        print(
            f"{PROG}: serving {options.atlas} on "
            f"http://{options.host}:{options.port}{options.path} ({guard})",
            file=sys.stderr,
        )
    run(options)
    return EXIT_OK


# ---------------------------------------------------------------------------
# keeping the atlas current


def cmd_hook(args: argparse.Namespace) -> int:
    """Manage the git side: the hooks, the merge driver and the rebuild they run."""
    action = getattr(args, "hook_command", None)
    if not action:
        raise UsageError(
            "hook takes an action: install, status, uninstall, run or merge. "
            f"start with {PROG} hook install"
        )
    return _HOOK_ACTIONS[action](args)


def _hook_install(args: argparse.Namespace) -> int:
    from rosio import history

    report = history.install(
        args.repo,
        python=args.python,
        workspace=args.workspace,
        out=args.out,
        config=args.config,
        force=args.force,
        dry_run=args.dry_run,
        measure=args.measure,
    )
    return _answer(args, report)


def _hook_status(args: argparse.Namespace) -> int:
    from rosio import history

    return _answer(args, history.status(args.repo))


def _hook_uninstall(args: argparse.Namespace) -> int:
    from rosio import history

    return _answer(args, history.uninstall(args.repo, force=args.force, dry_run=args.dry_run))


def _hook_run(args: argparse.Namespace) -> int:
    """The rebuild a hook runs. It always exits zero, whatever the extraction did.

    A tool that can refuse a commit is uninstalled the first time it is wrong, so the
    failure is a line on stderr and the exit code stays clean.
    """
    from rosio import history
    from rosio.model import dumps

    try:
        record = history.run_rebuild(args.repo, event=args.event, reuse=args.reuse)
    except RosioError as exc:
        print(f"{PROG}: {exc}", file=sys.stderr)
        return EXIT_OK
    if args.json:
        print(dumps({"verb": "hook run", "ok": record["ok"], "data": record}), end="")
    elif not args.quiet:
        seconds = record["volatile"]["seconds"]
        if record.get("left_tracked"):
            print(
                f"{PROG} hook {record['event']}: {record['atlas']} is tracked by git, so the "
                f"hook left it as committed; run {PROG} hook run to rebuild it, then commit it"
            )
        elif record["ok"]:
            how = "atlas read back from the build cache" if record["cached"] else "atlas rebuilt"
            print(f"{PROG} hook {record['event']}: {how} in {seconds}s")
            dropped = record.get("dropped_layers") or []
            if dropped:
                print(
                    f"{PROG} hook {record['event']}: the rebuild holds no {' or '.join(dropped)} "
                    f"layer, which the replaced atlas had; run {PROG} probe again to restate it",
                    file=sys.stderr,
                )
        else:
            print(
                f"{PROG} hook {record['event']}: the rebuild failed after {seconds}s "
                f"({record['error'] or 'exit ' + str(record['exit'])}), your work is unaffected",
                file=sys.stderr,
            )
    return EXIT_OK


def _hook_merge(args: argparse.Namespace) -> int:
    """The merge driver. Exits zero when it produced one atlas, which is the point."""
    from rosio import mergetool
    from rosio.model import dumps

    result = mergetool.merge_files(
        args.base, args.ours, args.theirs, out=args.result, path=args.path
    )
    if args.json:
        print(dumps({"verb": "hook merge", "ok": True, "data": result.counts}), end="")
    elif not args.quiet:
        for line in result.lines():
            print(line)
    return EXIT_OK


_HOOK_ACTIONS: dict[str, Callable[[argparse.Namespace], int]] = {
    "install": _hook_install,
    "status": _hook_status,
    "uninstall": _hook_uninstall,
    "run": _hook_run,
    "merge": _hook_merge,
}


def cmd_bundle(args: argparse.Namespace) -> int:
    """Carry the evidence to another checkout without collecting the system again."""
    from rosio import bundle
    from rosio.query import Answer

    action = args.bundle_command
    if action == "create":
        target = bundle.create(
            args.workspace,
            atlas=args.atlas,
            out=args.archive,
            out_dir=args.out,
            include=args.include,
            note=args.note,
        )
        return _answer(
            args,
            Answer(
                "bundle create", data={"written": str(target)}, lines=[f"rosio bundle: {target}"]
            ),
        )
    if action == "open":
        text = bundle.open_bundle(args.file, into=args.into, force=args.force)
        return _answer(args, Answer("bundle open", lines=text.splitlines(), data={"summary": text}))
    raise UsageError("bundle takes an action: create or open")


def cmd_migrate(args: argparse.Namespace) -> int:
    """Plan, apply or verify a move over the workspace's source packages."""
    import os
    from pathlib import Path

    from rosio import migrate
    from rosio.config import DEFAULT_OUT_DIR
    from rosio.migrate import report
    from rosio.query import Answer

    action = args.migrate_command
    if action not in {"plan", "apply", "verify"}:
        raise UsageError("migrate takes an action: plan, apply or verify")
    start = args.start or os.environ.get("ROS_DISTRO")
    if not start:
        raise UsageError("pass --from, ROS_DISTRO is not set")
    workspace = Path(args.workspace).expanduser()
    if action == "verify":
        from rosio.errors import EXIT_ERROR
        from rosio.migrate.verify import lines as verify_lines
        from rosio.migrate.verify import verify_workspace

        data = verify_workspace(workspace, start, args.target, args.config, args.image)
        _answer(args, Answer("migrate verify", ok=data["ok"], data=data, lines=verify_lines(data)))
        return EXIT_OK if data["ok"] else EXIT_ERROR
    changes, root = migrate.plan_workspace(workspace, start, args.target, args.config)
    data = report.data(changes, start, args.target)
    data["root"] = str(root)
    lines = report.lines(changes, start, args.target)
    if action == "apply":
        diff = migrate.apply(changes, root, force=args.force)
        out = Path(args.out or DEFAULT_OUT_DIR)
        out = out if out.is_absolute() else workspace / out
        written = out / "migrate" / f"{start}-to-{args.target}.diff"
        rewritten = sum(1 for change in changes if change.automatic)
        if not diff:
            # A second apply rewrites nothing, and the diff of the first is the record.
            kept = f", the earlier diff stays in {written}" if written.is_file() else ""
            data["diff"] = str(written) if kept else ""
            lines = [f"rosio migrate apply: nothing to rewrite{kept}"]
            return _answer(args, Answer(f"migrate {action}", data=data, lines=lines))
        written.parent.mkdir(parents=True, exist_ok=True)
        written.write_text(diff, encoding="utf-8")
        data["diff"] = str(written)
        lines = [
            f"rosio migrate apply: {rewritten} rewrites, diff in {written}",
            "plan again to see what is left by hand",
        ]
    return _answer(args, Answer(f"migrate {action}", data=data, lines=lines))


HANDLERS: dict[str, Callable[[argparse.Namespace], int]] = {
    "version": cmd_version,
    "extract": cmd_extract,
    "probe": cmd_probe,
    "drift": cmd_drift,
    "explain": cmd_explain,
    "param": cmd_param,
    "query": cmd_query,
    "path": cmd_path,
    "diff": cmd_diff,
    "unresolved": cmd_unresolved,
    "graph": cmd_graph,
    "render": cmd_render,
    "install": cmd_install,
    "uninstall": cmd_uninstall,
    "serve": cmd_serve,
    "hook": cmd_hook,
    "stamps": cmd_stamps,
    "init": cmd_init,
    "bundle": cmd_bundle,
    "migrate": cmd_migrate,
}

IMPLEMENTED = frozenset(HANDLERS)
"""Every verb in the closed set does the work. Nothing here prints a milestone."""


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(list(argv) if argv is not None else None)

    if not args.command:
        parser.print_help()
        return EXIT_USAGE

    handler = HANDLERS[args.command]
    try:
        code = handler(args)
        # Flushed here so a reader that already left breaks the pipe inside this try and
        # not in the interpreter's own flush at exit, which prints a traceback.
        sys.stdout.flush()
        return code
    except RosioError as exc:
        print(f"{PROG}: {exc}", file=sys.stderr)
        return exit_code_for(exc)
    except KeyboardInterrupt:
        return 130
    except BrokenPipeError:
        # A reader such as `head` stopped reading. Point stdout at devnull so the flush at
        # exit does not raise again, and exit the way a shell reports SIGPIPE.
        devnull = os.open(os.devnull, os.O_WRONLY)
        os.dup2(devnull, sys.stdout.fileno())
        return 141


def commands() -> list[str]:
    return sorted(HANDLERS)
