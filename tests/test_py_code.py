"""The python half of the coded layer, checked against the committed expectations.

The comparison drops the advisory keys (prose and line hints) and nothing else, so a record
that loses an endpoint, invents a topic or fabricates a qos depth fails here.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rosio import ids, join, model, schema
from rosio.collect import CollectContext, py_code
from rosio.config import Config, RootSpec
from rosio.workspace import Workspace

WS = "ws_code"
EXPECTED = "expected/code"
PY_FILES = (
    "px_literal_node.json",
    "px_param_node.json",
    "px_tf_node.json",
    "px_wrappers_node.json",
)


@pytest.fixture(scope="module")
def ws_root(fixtures_dir: Path) -> Path:
    return fixtures_dir / WS


@pytest.fixture(scope="module")
def index(fixtures_dir: Path) -> dict:
    return json.loads((fixtures_dir / EXPECTED / "index.json").read_text())


def _expected(fixtures_dir: Path, name: str) -> dict:
    return json.loads((fixtures_dir / EXPECTED / name).read_text())


def _actual(ws_root: Path, relative: str) -> dict:
    return py_code.analyse_file(ws_root / relative, ws_root)


def _records(fixtures_dir: Path, ws_root: Path) -> list[tuple[dict, dict]]:
    pairs = []
    for name in PY_FILES:
        want = _expected(fixtures_dir, name)
        pairs.append((want, _actual(ws_root, want["path"])))
    return pairs


@pytest.mark.parametrize("name", PY_FILES)
def test_record_matches_expectation(fixtures_dir: Path, ws_root: Path, name: str) -> None:
    want = _expected(fixtures_dir, name)
    got = _actual(ws_root, want["path"])
    assert py_code.comparable(got) == py_code.comparable(want)


def test_totals_match_the_python_half_of_the_index(
    fixtures_dir: Path, ws_root: Path, index: dict
) -> None:
    keys = (
        "endpoints",
        "publishers",
        "subscribers",
        "timers",
        "service_servers",
        "service_clients",
        "parameters",
    )
    totals = dict.fromkeys(keys, 0)
    nodes = unresolved = wrapped = broadcasts = lookups = 0
    for _, got in _records(fixtures_dir, ws_root):
        for key in totals:
            totals[key] += got["counts"][key]
        nodes += len(got["nodes"])
        unresolved += len(got["unresolved"])
        wrapped += got["counts"].get("via_known_wrapper", 0)
        broadcasts += got["counts"].get("tf_broadcasts", 0)
        lookups += got["counts"].get("tf_lookups", 0)
    want = index["per_language"]["python"]
    assert totals["endpoints"] == want["endpoints"]
    assert totals["publishers"] == want["publishers"]
    assert totals["subscribers"] == want["subscribers"]
    assert totals["service_servers"] == want["service_servers"]
    assert totals["service_clients"] == want["service_clients"]
    assert totals["timers"] == want["timers"]
    assert totals["parameters"] == want["parameters"]
    assert unresolved == want["unresolved"]
    assert nodes == want["nodes"]
    assert (broadcasts, lookups, wrapped) == (2, 4, 7)


def test_unresolved_inventory_matches(fixtures_dir: Path, ws_root: Path, index: dict) -> None:
    want = sorted(
        (entry["file"], entry["want"], entry["expr"], entry["reason"])
        for entry in index["unresolved_inventory"]
        if entry["file"].endswith(".py")
    )
    got = sorted(
        (record["path"], entry["want"], entry["expr"], entry["reason"])
        for _, record in _records(fixtures_dir, ws_root)
        for entry in record["unresolved"]
    )
    assert got == want


def test_nothing_is_invented_for_an_unresolved_name(fixtures_dir: Path, ws_root: Path) -> None:
    for _, record in _records(fixtures_dir, ws_root):
        for endpoint in record["endpoints"]:
            if endpoint["name_state"] == "unresolved":
                assert endpoint["name"] is None
                assert "topic_id" not in endpoint
                assert endpoint["unresolved_ref"].split(":", 1)[0] in py_code.WANTS


def test_qos_is_never_defaulted(fixtures_dir: Path, ws_root: Path) -> None:
    """A policy the source did not state stays null, a stated one keeps its own value, and a
    `QoSProfile` states what rclpy fills in for a policy it is not given."""
    record = _actual(ws_root, "src/px_nodes/px_nodes/literal_node.py")
    by_name = {item["name"]: item for item in record["endpoints"] if item["kind"] == "publisher"}
    assert by_name["/px/odom"]["qos"]["depth"] == 5
    assert by_name["/px/odom"]["qos"]["reliability"] is None
    assert by_name["/px/ready"]["qos"]["history"] == "keep_last"
    assert by_name["/px/ready"]["qos"]["durability"] == "transient_local"
    services = [item for item in record["endpoints"] if item["kind"].startswith("service")]
    assert all(item["qos"] is None for item in services)


def test_a_stated_node_name_is_a_fact(ws_root: Path) -> None:
    record = _actual(ws_root, "src/px_nodes/px_nodes/literal_node.py")
    node = record["nodes"][0]
    assert (node["name"], node["name_source"], node["node_id"]) == ("px_literal", "literal", None)
    assert node["symbol_id"] == ids.build_sym(
        "px_nodes", "px_nodes/literal_node.py", "px_nodes.literal_node.LiteralNode.__init__"
    )


def test_executable_comes_from_the_entry_point_not_the_module(ws_root: Path) -> None:
    record = _actual(ws_root, "src/px_nodes/px_nodes/wrappers_node.py")
    assert record["executables"] == ["px_health"]


def test_ids_carry_no_absolute_path(fixtures_dir: Path, ws_root: Path) -> None:
    for _, record in _records(fixtures_dir, ws_root):
        text = json.dumps(record)
        assert str(ws_root) not in text
        assert "/tmp" not in text


def test_two_reads_give_the_same_record(fixtures_dir: Path, ws_root: Path) -> None:
    first = json.dumps(_records(fixtures_dir, ws_root), sort_keys=True)
    second = json.dumps(_records(fixtures_dir, ws_root), sort_keys=True)
    assert first == second


def test_a_file_without_ros_content_stays_empty(ws_root: Path) -> None:
    record = _actual(ws_root, "src/px_nodes/px_nodes/tuning.py")
    assert record["nodes"] == []
    assert record["endpoints"] == []
    assert record["unresolved"] == []


def test_unparseable_source_raises_a_collector_error() -> None:
    from rosio.errors import CollectorError

    with pytest.raises(CollectorError):
        py_code.analyse_source(
            "def broken(:\n", path="a.py", package="p", package_relative_path="a.py"
        )


IDIOMS = """
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

SOURCES = (
    ("/base/estop", Bool, True),
    ("/base/speed", Float32, False),
)

DEFAULTS = {"cmd_topic": "/cmd_vel_safe", "rate_hz": 20.0}


class Idioms(Node):
    def __init__(self):
        super().__init__("idioms")
        d = self.declare_parameter
        for name, value in DEFAULTS.items():
            d(name, value)

        def gp(n):
            return self.get_parameter(n).value

        self.cmd = self.create_publisher(Twist, str(gp("cmd_topic")).strip(), 10)
        for topic, kind, latched in SOURCES:
            self.create_subscription(kind, topic, self._on, 10)

    def _on(self, msg):
        pass
"""


def _idioms() -> dict:
    return py_code.analyse_source(
        IDIOMS, path="src/p/p/idioms.py", package="p", package_relative_path="p/idioms.py"
    )


def test_a_parameter_accessor_helper_resolves_the_name() -> None:
    """`def gp(n): return self.get_parameter(n).value` states the name at the call site."""
    record = _idioms()
    publisher = next(item for item in record["endpoints"] if item["kind"] == "publisher")
    assert publisher["name"] == "/cmd_vel_safe"
    assert publisher["from_parameter"] == "cmd_topic"


def test_a_bound_declare_parameter_alias_is_followed() -> None:
    names = [item["name"] for item in _idioms()["parameters"]]
    assert names == ["cmd_topic", "rate_hz"]


def test_a_source_table_keeps_the_message_type_of_each_row() -> None:
    record = _idioms()
    rows = [
        (item["name"], item["type"]) for item in record["endpoints"] if item["kind"] == "subscriber"
    ]
    assert rows == [("/base/estop", "std_msgs/msg/Bool"), ("/base/speed", "std_msgs/msg/Float32")]


IN_MAIN = """
def main():
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from nav2_msgs.action import NavigateToPose
    from std_msgs.msg import Bool

    class Worker(Node):
        def __init__(self):
            super().__init__("worker")
            self.declare_parameter("out_topic", "/worker/out")
            self.pub = self.create_publisher(Bool, "/worker/out", 5)
            self.nav = ActionClient(self, NavigateToPose, "/navigate_to_pose")

    rclpy.init()
    rclpy.spin(Worker())
"""


def test_a_node_class_defined_inside_main_is_read() -> None:
    """One node on the reference stack lives inside `def main()` with local imports."""
    record = py_code.analyse_source(
        IN_MAIN, path="src/p/p/inmain.py", package="p", package_relative_path="p/inmain.py"
    )
    node = record["nodes"][0]
    assert (node["name"], node["name_source"]) == ("worker", "literal")
    assert node["symbol"] == "p.inmain.main.Worker.__init__"
    assert [item["name"] for item in record["parameters"]] == ["out_topic"]
    assert record["counts"]["publishers"] == 1
    assert record["counts"]["service_clients"] == 0
    assert [item["kind"] for item in record["endpoints"] if "navigate_to_pose" in item["raw"]] == [
        "action_client"
    ]


def test_an_rclpy_action_client_is_one_endpoint_of_the_action_type() -> None:
    """Read as the C++ reader reads `rclcpp_action`, not as the five `_action/` endpoints
    rcl_action derives from the name."""
    record = py_code.analyse_source(
        IN_MAIN, path="src/p/p/inmain.py", package="p", package_relative_path="p/inmain.py"
    )
    assert not [item for item in record["endpoints"] if "_action/" in str(item["name"])]
    (action,) = [item for item in record["endpoints"] if item["kind"] == "action_client"]
    assert action["type"] == "nav2_msgs/action/NavigateToPose"


def test_a_table_row_keeps_the_message_class_of_the_row() -> None:
    """The message class of a table row survives the loop over the table."""
    source = (
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool, Float32\n"
        "SENSORS = {\n"
        '    "a": {"topic": "/a/in", "msg_type": Bool},\n'
        '    "b": {"topic": "/b/in", "msg_type": Float32},\n'
        "}\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        for name, cfg in SENSORS.items():\n"
        "            self.create_subscription(cfg['msg_type'], cfg['topic'], self.cb, 1)\n"
        "    def cb(self, msg):\n"
        "        pass\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/t.py", package="p", package_relative_path="p/t.py"
    )
    rows = [(item["name"], item["type"]) for item in record["endpoints"]]
    assert rows == [("/a/in", "std_msgs/msg/Bool"), ("/b/in", "std_msgs/msg/Float32")]


def test_two_node_classes_in_one_file_keep_their_own_parameter_defaults() -> None:
    """The first class must not lend its default to the second and rename its topic."""
    source = (
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class Alpha(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('alpha')\n"
        "        self.declare_parameter('out_topic', '/alpha/out')\n"
        "        self.p = self.create_publisher(Bool, self.get_parameter('out_topic').value, 1)\n"
        "class Beta(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('beta')\n"
        "        self.declare_parameter('out_topic', '/beta/out')\n"
        "        self.p = self.create_publisher(Bool, self.get_parameter('out_topic').value, 1)\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/two.py", package="p", package_relative_path="p/two.py"
    )
    assert [(item["owner_symbol"], item["default"]) for item in record["parameters"]] == [
        ("p.two.Alpha.__init__", "/alpha/out"),
        ("p.two.Beta.__init__", "/beta/out"),
    ]
    assert [(item["owner_symbol"], item["name"]) for item in record["endpoints"]] == [
        ("p.two.Alpha.__init__", "/alpha/out"),
        ("p.two.Beta.__init__", "/beta/out"),
    ]


def test_a_service_call_spelled_with_keywords_resolves() -> None:
    source = (
        "from rclpy.node import Node\n"
        "from std_srvs.srv import Trigger\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.c = self.create_client(srv_type=Trigger, srv_name='/reset')\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/k.py", package="p", package_relative_path="p/k.py"
    )
    endpoint = record["endpoints"][0]
    assert (endpoint["name"], endpoint["type"]) == ("/reset", "std_srvs/srv/Trigger")
    assert record["unresolved"] == []


def test_one_unreadable_file_does_not_empty_the_collector(tmp_path: Path) -> None:
    """A source ast.parse refuses with ValueError, not SyntaxError, is still one failure."""
    package = tmp_path / "src" / "p"
    (package / "p").mkdir(parents=True)
    (package / "package.xml").write_text('<package format="3"><name>p</name></package>')
    (package / "p" / "good.py").write_text(
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class A(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('a')\n"
        "        self.p = self.create_publisher(Bool, '/a/out', 1)\n"
    )
    (package / "p" / "nul.py").write_bytes(b'x = "a\x00b"\n')
    (package / "p" / "syntax.py").write_text("def broken(:\n")
    out = py_code.analyse_workspace(tmp_path, "src")
    assert [item["path"] for item in out["files"]] == ["src/p/p/good.py"]
    assert sorted(item["path"] for item in out["failures"]) == [
        "src/p/p/nul.py",
        "src/p/p/syntax.py",
    ]


def test_a_first_party_class_named_like_a_wrapper_states_nothing() -> None:
    """The near miss for the wrapper table: one short name is not one library."""
    source = (
        "from rclpy.node import Node\n"
        "class Subscriber:\n"
        "    def __init__(self, a, b, c):\n"
        "        self.a = a\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.s = Subscriber(self, 'x', '/looks/like/a/topic')\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/s.py", package="p", package_relative_path="p/s.py"
    )
    assert record["endpoints"] == []
    assert record["unresolved"] == []


def test_a_wrapper_spelled_through_another_library_states_nothing() -> None:
    source = (
        "from rclpy.node import Node\n"
        "import mylib.messaging as messaging\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.s = messaging.Subscriber(self, 'x', '/y')\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/m.py", package="p", package_relative_path="p/m.py"
    )
    assert record["endpoints"] == []


def test_a_member_of_one_class_is_not_lent_to_another() -> None:
    """Two classes in one file, one topic. The second states none and says so."""
    source = (
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class Alpha(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('alpha')\n"
        "        self.topic = '/alpha/out'\n"
        "        self.p = self.create_publisher(Bool, self.topic, 1)\n"
        "class Beta(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('beta')\n"
        "        self.p = self.create_publisher(Bool, self.topic, 1)\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/two.py", package="p", package_relative_path="p/two.py"
    )
    rows = [(item["owner_symbol"], item["name"]) for item in record["endpoints"]]
    assert rows == [("p.two.Alpha.__init__", "/alpha/out"), ("p.two.Beta.__init__", None)]
    assert [entry["want"] for entry in record["unresolved"]] == ["topic_name"]


def test_a_subclass_keeps_the_members_its_base_class_sets() -> None:
    source = (
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class Base(Node):\n"
        "    def __init__(self, name):\n"
        "        super().__init__(name)\n"
        "        self.topic = '/base/out'\n"
        "class Child(Base):\n"
        "    def __init__(self):\n"
        "        super().__init__('child')\n"
        "        self.p = self.create_publisher(Bool, self.topic, 1)\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/i.py", package="p", package_relative_path="p/i.py"
    )
    assert [item["name"] for item in record["endpoints"]] == ["/base/out"]


def test_an_environment_name_is_refused_rather_than_guessed() -> None:
    source = (
        "import os\n"
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.p = self.create_publisher(Bool, os.environ['TOPIC'], 1)\n"
    )
    record = py_code.analyse_source(
        source, path="src/p/p/e.py", package="p", package_relative_path="p/e.py"
    )
    endpoint = record["endpoints"][0]
    assert endpoint["name"] is None
    assert record["unresolved"][0]["want"] == "topic_name"


# ---------------------------------------------------------------------------
# the functional node form


CREATE_NODE = """
import rclpy
from std_msgs.msg import Bool


def main():
    rclpy.init()
    node = rclpy.create_node("ready_gate")
    node.create_publisher(Bool, "/mapping/ready", 1)
"""


def _created(source: str = CREATE_NODE) -> dict:
    return py_code.analyse_source(
        source, path="src/p/p/gate.py", package="p", package_relative_path="p/gate.py"
    )


def test_the_functional_node_form_states_a_name() -> None:
    """`rclpy.create_node("literal")` is a node, and five sites spell it that way."""
    record = _created()
    node = record["nodes"][0]
    assert (node["name"], node["name_source"], node["api"]) == (
        "ready_gate",
        "literal",
        "rclpy.create_node",
    )
    assert node["node_id"] is None
    assert node["symbol"] == "p.gate.main"
    assert node["symbol_id"] == ids.build_sym("p", "p/gate.py", "p.gate.main")


def test_the_endpoints_of_a_created_node_share_its_owner() -> None:
    record = _created()
    endpoint = record["endpoints"][0]
    assert (endpoint["name"], endpoint["owner_symbol"]) == ("/mapping/ready", "p.gate.main")
    assert record["nodes"][0]["symbol"] == endpoint["owner_symbol"]


def test_a_created_node_name_that_is_not_a_literal_is_refused() -> None:
    source = "import rclpy\ndef main(label):\n    node = rclpy.create_node(label)\n"
    record = _created(source)
    node = record["nodes"][0]
    assert (node["name"], node["name_source"]) == (None, "argv")
    assert [entry["want"] for entry in record["unresolved"]] == ["node_name"]


AUTO_DECLARE = """
import rclpy
from rclpy.node import Node


class SafetyStop(Node):
    def __init__(self):
        super().__init__("safety_stop", automatically_declare_parameters_from_overrides=%s)
        self.distance = self.get_parameter("stop_distance").value
"""


@pytest.mark.parametrize(
    ("value", "stated"), [("True", True), ("flag", "unknown"), ("False", None)]
)
def test_a_node_that_declares_from_overrides_says_so(value: str, stated) -> None:
    record = _created(AUTO_DECLARE % value)
    (node,) = record["nodes"]
    assert node.get("auto_declare") == stated


def test_a_created_node_that_allows_undeclared_parameters_says_so() -> None:
    source = (
        "import rclpy\ndef main():\n"
        "    node = rclpy.create_node('gate', allow_undeclared_parameters=True)\n"
    )
    assert _created(source)["nodes"][0]["auto_declare"] is True


def test_a_create_node_of_another_library_is_not_a_ros_node() -> None:
    source = "import graphlib\ndef main():\n    graphlib.create_node('x')\n"
    assert _created(source)["nodes"] == []


# ---------------------------------------------------------------------------
# first party wrappers, discovered over the workspace


REPORTER = """
from diagnostic_updater import Updater


class Reporter:
    def __init__(self, node: "Node", hardware_id: str = "fx"):
        self.updater = Updater(node, period=1.0)
        self.updater.setHardwareID(hardware_id)
"""

USER = """
from rclpy.node import Node
from px_wrap.report import Reporter


class Worker(Node):
    def __init__(self):
        super().__init__("worker")
        self._diag = Reporter(self, hardware_id="worker")
"""


def _package(tmp_path: Path, files: dict[str, str], name: str = "px_wrap") -> Path:
    package = tmp_path / "src" / name
    (package / name).mkdir(parents=True)
    (package / "package.xml").write_text(f'<package format="3"><name>{name}</name></package>')
    (package / "setup.py").write_text(
        "from setuptools import setup\n"
        f"setup(name='{name}', entry_points={{'console_scripts': ['worker = {name}.user:main']}})\n"
    )
    for relative, text in files.items():
        (package / name / relative).write_text(text)
    return tmp_path


def _atlas(root: Path) -> tuple[model.Atlas, py_code.PyCodeCollector]:
    roots = (RootSpec("src", str(root / "src")),)
    ws = Workspace.discover(list(roots))
    builder = model.AtlasBuilder()
    collector = py_code.PyCodeCollector()
    collector.collect(CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder))
    return builder.build(), collector


def test_a_first_party_wrapper_class_is_discovered_across_two_modules(tmp_path: Path) -> None:
    """The construction and the class that makes it a wrapper are in two different files."""
    root = _package(tmp_path, {"report.py": REPORTER, "user.py": USER})
    atlas, collector = _atlas(root)
    assert [row.symbol for row in collector.discovered] == ["px_wrap.report.Reporter"]
    sym = ids.build_sym("px_wrap", "px_wrap/user.py", "px_wrap.user.Worker.__init__")
    fact = atlas.entities[sym].fact("coded")
    endpoints = fact.values["endpoints"]
    assert [(item["kind"], item["name"]) for item in endpoints] == [("publisher", "/diagnostics")]
    assert endpoints[0]["via_known_wrapper"] == "px_wrap.report.Reporter"
    assert endpoints[0]["qos"] is None
    assert "topic:/diagnostics" in atlas.entities


def test_a_class_that_takes_no_node_stays_quiet(tmp_path: Path) -> None:
    """The near miss: it holds an updater it was handed, so it creates nothing itself."""
    holder = REPORTER.replace('node: "Node"', "updater").replace(
        "self.updater = Updater(node, period=1.0)", "self.updater = updater"
    )
    root = _package(tmp_path, {"report.py": holder, "user.py": USER})
    atlas, collector = _atlas(root)
    assert collector.discovered == ()
    assert "topic:/diagnostics" not in atlas.entities


def test_a_node_class_is_never_promoted_to_a_wrapper(tmp_path: Path) -> None:
    """A node that constructs the updater itself owns the endpoint, and wraps nothing."""
    source = (
        "from diagnostic_updater import Updater\n"
        "from rclpy.node import Node\n"
        "class Direct(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('direct')\n"
        "        self.updater = Updater(self)\n"
    )
    root = _package(tmp_path, {"user.py": source})
    _atlas_out, collector = _atlas(root)
    assert collector.discovered == ()


def test_a_relative_import_reaches_the_same_wrapper(tmp_path: Path) -> None:
    """`from .report import Reporter` has to name the class the absolute spelling names."""
    relative = USER.replace("from px_wrap.report import Reporter", "from .report import Reporter")
    root = _package(tmp_path, {"report.py": REPORTER, "user.py": relative})
    out = py_code.analyse_workspace(root, "src")
    names = [item["name"] for record in out["files"] for item in record["endpoints"]]
    assert names.count("/diagnostics") == 2
    assert out["wrappers"] == ["px_wrap.report.Reporter"]


def test_the_twelve_diagnostics_publishers_need_two_passes(tmp_path: Path) -> None:
    """One pass over the same tree, with no discovery, states no /diagnostics at all."""
    root = _package(tmp_path, {"report.py": REPORTER, "user.py": USER})
    out = py_code.analyse_workspace(root, "src", discover=False)
    names = [item["name"] for record in out["files"] for item in record["endpoints"]]
    assert names.count("/diagnostics") == 1
    with_discovery = py_code.analyse_workspace(root, "src")
    names = [item["name"] for record in with_discovery["files"] for item in record["endpoints"]]
    assert names.count("/diagnostics") == 2
    assert with_discovery["wrappers"] == ["px_wrap.report.Reporter"]


# ---------------------------------------------------------------------------
# the collector


def test_the_collector_writes_a_fact_per_file_and_per_symbol(ws_root: Path) -> None:
    atlas, _collector = _atlas(ws_root)
    file_id = ids.build_file("px_nodes", "px_nodes/literal_node.py")
    values = atlas.entities[file_id].fact("coded").values
    assert values["language"] == "python"
    assert values["parsed"] is True
    assert [row["name"] for row in values["nodes"]] == ["px_literal"]
    sym = ids.build_sym(
        "px_nodes", "px_nodes/literal_node.py", "px_nodes.literal_node.LiteralNode.__init__"
    )
    fact = atlas.entities[sym].fact("coded")
    assert [row["name"] for row in fact.values["nodes"]] == ["px_literal"]
    assert any(item["name"] == "/px/odom" for item in fact.values["endpoints"])
    assert sym in [edge.dst for edge in atlas.out_edges(file_id) if edge.kind == "defines"]


def test_a_symbol_fact_carries_the_record_block_and_nothing_else(ws_root: Path) -> None:
    """AGENTS section 2.2. The block is the whole of what a coded collector hands the join,
    so a key outside it is written for nobody: the join skips it and every later pass reads
    the join. The symbol fact spelled its node rows `node` and carried a `parameter_callbacks`
    list no pass has ever read, and both are gone."""
    atlas, _collector = _atlas(ws_root)
    facts = [
        entity.fact("coded")
        for key, entity in sorted(atlas.entities.items())
        if key.startswith("sym:")
    ]
    assert facts and all(fact is not None for fact in facts)
    written = {key for fact in facts for key in fact.values}
    assert written <= set(join.RECORD_KEYS), sorted(written - set(join.RECORD_KEYS))
    assert "nodes" in written


def test_the_collector_output_validates_and_is_stable(ws_root: Path) -> None:
    atlas, _collector = _atlas(ws_root)
    assert schema.validate(json.loads(atlas.to_bytes())) == []
    again, _ = _atlas(ws_root)
    assert atlas.canonical_bytes() == again.canonical_bytes()


def test_a_relative_code_name_mints_no_topic(tmp_path: Path) -> None:
    source = (
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.p = self.create_publisher(Bool, 'relative_out', 1)\n"
    )
    root = _package(tmp_path, {"user.py": source})
    atlas, _collector = _atlas(root)
    assert not [name for name in atlas.entities if name.startswith("topic:")]


def test_a_blind_spot_lands_on_the_symbol_that_owns_it(tmp_path: Path) -> None:
    source = (
        "import os\n"
        "from rclpy.node import Node\n"
        "from std_msgs.msg import Bool\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.p = self.create_publisher(Bool, os.environ['TOPIC'], 1)\n"
    )
    root = _package(tmp_path, {"user.py": source})
    atlas, _collector = _atlas(root)
    entry = atlas.unresolved[0]
    assert entry.want == "topic_name"
    assert entry.owner == ids.build_sym("px_wrap", "px_wrap/user.py", "px_wrap.user.N.__init__")
    assert entry.role == "py_source"
    assert atlas.entities[entry.owner].unresolved == [entry.id]


def test_a_parameter_name_built_at_run_time_is_a_param_name_blind_spot(tmp_path: Path) -> None:
    """The join reads `param_name` blind spots to decide which keys a node may declare, so a
    python loop over names recorded under `other` made every key it is given undeclared."""
    source = (
        "from rclpy.node import Node\n"
        "class N(Node):\n"
        "    def __init__(self):\n"
        "        super().__init__('n')\n"
        "        self.declare_parameter('stop_distance', 0.5)\n"
        "        for name in self._names():\n"
        "            self.declare_parameter(name, 0.0)\n"
    )
    root = _package(tmp_path, {"user.py": source})
    atlas, _collector = _atlas(root)
    assert [(entry.want, entry.expr) for entry in atlas.unresolved] == [("param_name", "name")]
    assert atlas.counters["resolution_rate"]["param_name"] == 0.5


def test_one_unreadable_file_is_one_run_exception(tmp_path: Path) -> None:
    root = _package(tmp_path, {"user.py": USER, "broken.py": "def broken(:\n"})
    atlas, _collector = _atlas(root)
    run = next(entity for entity in atlas.entities.values() if entity.kind == "run")
    assert [row["path"] for row in run.attrs["exceptions"]] == [
        ids.build_file("px_wrap", "px_wrap/broken.py")
    ]
    assert ids.build_file("px_wrap", "px_wrap/user.py") in atlas.entities


def test_a_file_that_does_not_parse_is_still_a_file_of_its_package(tmp_path: Path) -> None:
    """A file this layer refused to read is stated as refused, not as one holding nothing."""
    root = _package(tmp_path, {"user.py": USER, "broken.py": "def broken(:\n"})
    atlas, _collector = _atlas(root)
    broken = ids.build_file("px_wrap", "px_wrap/broken.py")
    assert atlas.entities[broken].attrs["loadable"] is False
    assert atlas.entities[broken].attrs["refused_because"] == "py_parse_error"
    assert atlas.entities[broken].fact("coded") is None
    assert atlas.entities[broken].layers == []
    assert broken in [edge.dst for edge in atlas.out_edges(ids.build_pkg("px_wrap"))]
    entries = [item for item in atlas.unresolved if item.owner == broken]
    assert {entry.want for entry in entries} == {"node_name", "topic_name"}
    assert {entry.reason for entry in entries} == {"py_parse_error"}
    assert atlas.entities[broken].unresolved == sorted(entry.id for entry in entries)


def test_no_absolute_path_reaches_the_atlas(ws_root: Path) -> None:
    atlas, _collector = _atlas(ws_root)
    assert str(ws_root) not in atlas.to_bytes().decode("utf-8")


def test_the_collector_states_the_frames_of_a_broadcast_and_no_tf_parent(ws_root: Path) -> None:
    """One producer per fact. The tf_parent edge is the join's, which is the only pass that
    knows the profile and the node a broadcast runs in."""
    atlas, _collector = _atlas(ws_root)
    assert "frame:base_link" in atlas.entities
    assert "frame:px_lidar" in atlas.entities
    assert [edge for edge in atlas.edges.values() if edge.kind == "tf_parent"] == []


# ---------------------------------------------------------------------------
# the three source patterns


MODULE_LEVEL = """
import rclpy
from std_msgs.msg import Bool

rclpy.init()
node = rclpy.create_node("readiness_gate")
node.declare_parameter("timeout_s", 180.0)
node.create_subscription(Bool, "/mapping/ready", print, 10)
"""


def test_a_node_written_at_module_level_is_read() -> None:
    """A script with no class and no main is a node like any other, and walking only the
    definitions of the file states nothing at all about it."""
    record = py_code.analyse_source(
        MODULE_LEVEL, path="src/p/p/gate.py", package="p", package_relative_path="p/gate.py"
    )
    node = record["nodes"][0]
    assert (node["name"], node["name_source"], node["symbol"]) == (
        "readiness_gate",
        "literal",
        "p.gate",
    )
    assert [(item["kind"], item["name"]) for item in record["endpoints"]] == [
        ("subscriber", "/mapping/ready")
    ]
    assert [item["name"] for item in record["parameters"]] == ["timeout_s"]
    assert record["endpoints"][0]["owner_symbol"] == "p.gate"


BASE_MODULE = """
from rclpy.node import Node


class BridgeBase(Node):
    def __init__(self, name):
        super().__init__(name)
"""

DERIVED_MODULE = """
from std_msgs.msg import Bool

from px_wrap.base import BridgeBase


class Bridge(BridgeBase):
    def __init__(self):
        super().__init__("yard_bridge")
        self.pub = self.create_publisher(Bool, "/yard/state", 1)
"""


def test_a_class_whose_base_class_is_in_another_module_is_a_node(tmp_path: Path) -> None:
    """The base class of a node is regularly one module away, and until the workspace pass
    states which classes are nodes, the name such a class spells is lost."""
    root = _package(tmp_path, {"base.py": BASE_MODULE, "user.py": DERIVED_MODULE})
    out = py_code.analyse_workspace(root, "src")
    derived = next(item for item in out["files"] if item["path"].endswith("user.py"))
    assert [(row["name"], row["name_source"]) for row in derived["nodes"]] == [
        ("yard_bridge", "literal")
    ]


def test_a_class_whose_base_class_is_unknown_states_no_node(tmp_path: Path) -> None:
    """The near miss: a base this workspace does not define is not made a node."""
    root = _package(tmp_path, {"user.py": DERIVED_MODULE})
    out = py_code.analyse_workspace(root, "src")
    derived = next(item for item in out["files"] if item["path"].endswith("user.py"))
    assert derived["nodes"] == []


SHARED_PATH_NODE_BASE = """
from rclpy.node import Node


class Helper(Node):
    def __init__(self, name):
        super().__init__(name)
"""

SHARED_PATH_PLAIN_BASE = """
class Helper:
    def __init__(self, name):
        self.name = name
"""

SHARED_PATH_USER = """
from scripts.base import Helper


class PlainThing(Helper):
    def __init__(self):
        super().__init__("not_a_ros_node")
"""


def _shared_path_workspace(tmp_path: Path) -> Path:
    """Two packages spelling one module path, which is `scripts/`, `test/` and `setup.py`
    on every ament workspace."""
    for name, files in (
        ("pkg_a", {"base.py": SHARED_PATH_NODE_BASE}),
        ("pkg_b", {"base.py": SHARED_PATH_PLAIN_BASE, "user.py": SHARED_PATH_USER}),
    ):
        package = tmp_path / "src" / name
        (package / "scripts").mkdir(parents=True)
        (package / "package.xml").write_text(f'<package format="3"><name>{name}</name></package>')
        for relative, text in files.items():
            (package / "scripts" / relative).write_text(text)
    return tmp_path


def test_a_module_path_two_packages_spell_does_not_lend_one_its_node_base(
    tmp_path: Path,
) -> None:
    """A module path is package relative, so the base table is answered by the package that
    owns the name. Sharing it turns a plain class into a node another package never states."""
    out = py_code.analyse_workspace(_shared_path_workspace(tmp_path), "src")
    user = next(item for item in out["files"] if item["path"].endswith("pkg_b/scripts/user.py"))
    assert user["nodes"] == []


def test_the_base_table_is_keyed_by_the_package_that_states_it(tmp_path: Path) -> None:
    out = py_code.analyse_workspace(_shared_path_workspace(tmp_path), "src")
    assert py_code.base_key("pkg_a", "scripts.base.Helper") != py_code.base_key(
        "pkg_b", "scripts.base.Helper"
    )
    assert [item["path"] for item in out["files"] if item["nodes"]] == ["src/pkg_a/scripts/base.py"]


BRANCHED = """
from rclpy.node import Node
from std_msgs.msg import Bool


class N(Node):
    def __init__(self, sim):
        super().__init__("n")
        if sim:
            topic = "/sim/cmd"
        else:
            topic = "/real/cmd"
        self.pub = self.create_publisher(Bool, topic, 1)
"""


def _branched(source: str = BRANCHED) -> dict:
    return py_code.analyse_source(
        source, path="src/p/p/b.py", package="p", package_relative_path="p/b.py"
    )


def test_a_name_the_two_arms_bind_differently_is_a_blind_spot() -> None:
    """The dangerous one: taking the else arm states a topic that is wrong on the other
    branch, with nothing in the record saying a branch was involved."""
    record = _branched()
    endpoint = record["endpoints"][0]
    assert (endpoint["name"], endpoint["name_state"]) == (None, "unresolved")
    assert endpoint["condition"] == "sim"
    entry = record["unresolved"][0]
    assert (entry["want"], entry["expr"], entry["reason"]) == (
        "topic_name",
        "topic",
        "branch_dependent",
    )
    assert entry["condition"] == "sim"
    assert "topic:/real/cmd" not in str(record)


def test_a_member_the_two_arms_bind_differently_is_a_blind_spot() -> None:
    record = _branched(BRANCHED.replace("topic", "self._topic"))
    assert record["endpoints"][0]["name"] is None
    assert [entry["reason"] for entry in record["unresolved"]] == ["branch_dependent"]


def test_a_name_the_two_arms_agree_on_stays_resolved() -> None:
    """The near miss: both arms bind the same value, so the value holds either way."""
    record = _branched(BRANCHED.replace('"/sim/cmd"', '"/real/cmd"'))
    endpoint = record["endpoints"][0]
    assert (endpoint["name"], endpoint["name_state"]) == ("/real/cmd", "literal_absolute")
    assert record["unresolved"] == []


def test_a_name_only_one_arm_binds_keeps_the_value_it_states() -> None:
    source = BRANCHED.replace('        else:\n            topic = "/real/cmd"\n', "")
    record = _branched(source)
    assert record["endpoints"][0]["name"] == "/sim/cmd"


NAMESPACED = """
from rclpy.node import Node


class N(Node):
    def __init__(self):
        super().__init__("n", namespace="rig")
"""


def test_a_namespace_stated_in_code_is_kept() -> None:
    record = _branched(NAMESPACED)
    node = record["nodes"][0]
    assert (node["namespace"], node["namespace_source"]) == ("rig", "literal")
    assert "the declared layer" not in node["node_id_reason"]
    assert node["node_id"] is None


def test_a_namespace_that_is_not_a_literal_is_not_guessed() -> None:
    record = _branched(
        NAMESPACED.replace('namespace="rig"', "namespace=ns").replace(
            "def __init__(self):", "def __init__(self, ns):"
        )
    )
    node = record["nodes"][0]
    assert node["namespace_source"] == "unresolved"
    assert "namespace" not in node


ACTION_KEYWORDS = """
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose


class Runner(Node):
    def __init__(self):
        super().__init__("runner")
        self.server = ActionServer(
            node=self,
            action_type=NavigateToPose,
            action_name="/do_thing",
            execute_callback=self.run,
        )
        self.client = rclpy.action.ActionClient(
            node=self, action_type=NavigateToPose, action_name="/other"
        )

    def run(self, goal):
        return None
"""


def _keyword_record() -> dict:
    return py_code.analyse_source(
        ACTION_KEYWORDS,
        path="src/p/p/runner.py",
        package="p",
        package_relative_path="p/runner.py",
    )


def test_an_action_built_with_keyword_arguments_states_its_name() -> None:
    """`node`, `action_type` and `action_name` are all valid constructor parameters on
    humble. Read positionally, the construction stated no name at all and one action became
    five endpoints with none."""
    record = _keyword_record()
    served = [item["name"] for item in record["endpoints"] if item["kind"] == "action_server"]
    assert served == ["/do_thing"]
    assert not [item for item in record["endpoints"] if item["name"] is None]


def test_the_action_client_reads_its_keywords_too() -> None:
    record = _keyword_record()
    called = [item["name"] for item in record["endpoints"] if item["kind"] == "action_client"]
    assert called == ["/other"]


MESSAGE_FILTER_KEYWORDS = """
import message_filters
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2


class Fuser(Node):
    def __init__(self):
        super().__init__("fuser")
        self.left = message_filters.Subscriber(
            node=self, msg_type=PointCloud2, topic="/left/points", qos_profile=5
        )
"""


def test_a_message_filters_subscriber_built_with_keywords_states_its_topic() -> None:
    """The same assumption, in the other wrapper row whose caller supplies the name."""
    record = py_code.analyse_source(
        MESSAGE_FILTER_KEYWORDS,
        path="src/p/p/fuser.py",
        package="p",
        package_relative_path="p/fuser.py",
    )
    row = next(item for item in record["endpoints"] if item["kind"] == "subscriber")
    assert row["name"] == "/left/points"
    assert row["type"] == "sensor_msgs/msg/PointCloud2"
    assert row["qos"]["depth"] == 5
    assert row["via_known_wrapper"] == "message_filters.Subscriber"


NON_FINITE_DEFAULT = """
from rclpy.node import Node
from std_msgs.msg import Bool


class N(Node):
    def __init__(self):
        super().__init__('n')
        self.declare_parameter('good_a', 1)
        self.declare_parameter('gain', 1e400)
        self.declare_parameter('good_b', 2)
        self.p = self.create_publisher(Bool, '/px/out', 1)
"""


def test_a_default_json_cannot_hold_costs_one_blind_spot_and_not_the_pass(
    tmp_path: Path,
) -> None:
    """One `1e400` used to end the whole python pass and take every other file with it."""
    root = _package(tmp_path, {"user.py": NON_FINITE_DEFAULT, "other.py": USER})
    atlas, _collector = _atlas(root)
    symbol = ids.build_sym("px_wrap", "px_wrap/user.py", "px_wrap.user.N.__init__")
    entry = next(item for item in atlas.unresolved if item.owner == symbol)
    assert entry.want == "param_value"
    assert entry.expr == "1e400"
    assert "json has no infinity" in entry.reason
    assert atlas.entities[symbol].unresolved == [entry.id]
    assert "topic:/px/out" in atlas.entities
    assert ids.build_file("px_wrap", "px_wrap/other.py") in atlas.entities
    run = next(entity for entity in atlas.entities.values() if entity.kind == "run")
    assert run.attrs["exceptions"] == []
    assert model.dumps(atlas.to_json())


def test_the_named_parameters_of_the_same_file_survive_the_refused_one(tmp_path: Path) -> None:
    root = _package(tmp_path, {"user.py": NON_FINITE_DEFAULT})
    _atlas(root)
    record = py_code.analyse_source(
        NON_FINITE_DEFAULT,
        path="src/px_wrap/px_wrap/user.py",
        package="px_wrap",
        package_relative_path="px_wrap/user.py",
    )
    rows = {row["name"]: row for row in record["parameters"]}
    assert sorted(rows) == ["gain", "good_a", "good_b"]
    assert rows["good_a"]["default"] == 1
    assert rows["good_b"]["default"] == 2
    assert "default" not in rows["gain"] or rows["gain"]["default"] is None
    assert rows["gain"]["default_state"] == "unresolved"


DECLARE_MAP = """
from rclpy.node import Node


class N(Node):
    def __init__(self):
        super().__init__('n')
        self.declare_parameters('blk', {'a': 1.0, 'b': 1e400})
"""


def test_a_declare_parameters_map_is_read_the_way_the_cpp_half_reads_it() -> None:
    """The map form used to be dropped whole: neither name reached the atlas, while the C++
    half of the same call recorded both and one blind spot."""
    record = py_code.analyse_source(
        DECLARE_MAP,
        path="src/px_wrap/px_wrap/user.py",
        package="px_wrap",
        package_relative_path="px_wrap/user.py",
    )
    rows = {row["name"]: row for row in record["parameters"]}
    assert sorted(rows) == ["blk.a", "blk.b"]
    assert rows["blk.a"]["default"] == 1.0
    assert rows["blk.a"]["name_state"] == "map_key"
    assert rows["blk.a"]["map_namespace"] == "blk"
    assert rows["blk.b"]["default"] is None
    assert rows["blk.b"]["default_state"] == "unresolved"
    assert rows["blk.b"]["default_expr"] == "1e400"
    blind = [row for row in record["unresolved"] if row["want"] == "param_value"]
    assert [row["expr"] for row in blind] == ["1e400"]
    assert "json has no infinity" in blind[0]["reason"]


def test_a_declare_parameters_map_reaches_the_atlas(tmp_path: Path) -> None:
    root = _package(tmp_path, {"user.py": DECLARE_MAP})
    atlas, _collector = _atlas(root)
    symbol = ids.build_sym("px_wrap", "px_wrap/user.py", "px_wrap.user.N.__init__")
    names = [row["name"] for row in atlas.entities[symbol].facts[0].values["parameters"]]
    assert names == ["blk.a", "blk.b"]
    entry = next(item for item in atlas.unresolved if item.owner == symbol)
    assert entry.want == "param_value"
    assert schema.validate(atlas.to_json()) == []


def test_a_spread_in_a_declare_parameters_map_is_a_blind_spot() -> None:
    record = py_code.analyse_source(
        DECLARE_MAP.replace("'b': 1e400", "**self.rest"),
        path="src/px_wrap/px_wrap/user.py",
        package="px_wrap",
        package_relative_path="px_wrap/user.py",
    )
    assert [row["name"] for row in record["parameters"]] == ["blk.a"]
    assert [row["expr"] for row in record["unresolved"]] == ["self.rest"]


GUARDED = """
from rclpy.node import Node
from std_msgs.msg import Bool


class N(Node):
    def __init__(self, ready):
        super().__init__("n")
        self.publish_accel = self.declare_parameter("publish_acceleration", False).value
        if self.publish_accel:
            self.accel = self.create_publisher(Bool, "accel", 1)
        else:
            self.raw = self.create_publisher(Bool, "raw", 1)
        if ready:
            self.ready = self.create_publisher(Bool, "ready", 1)
"""


def test_a_parameter_guard_states_the_parameter_and_the_value_each_arm_needs() -> None:
    rows = {row["name"]: row for row in _branched(GUARDED)["endpoints"]}
    assert rows["accel"]["guard_parameters"] == [{"name": "publish_acceleration", "value": True}]
    assert rows["raw"]["guard_parameters"] == [{"name": "publish_acceleration", "value": False}]
    assert rows["ready"]["condition"] == "ready"
    assert "guard_parameters" not in rows["ready"]


def test_a_file_the_launch_walk_refused_is_still_read_and_stays_refused(tmp_path: Path) -> None:
    """A launch_testing file is named like a launch file and has no description to walk."""
    root = _package(tmp_path, {"user.py": USER, "report.py": REPORTER})
    roots = (RootSpec("src", str(root / "src")),)
    ws = Workspace.discover(list(roots))
    builder = model.AtlasBuilder()
    file_id = ws.file_id(str(root / "src" / "px_wrap" / "px_wrap" / "user.py"))
    builder.node(file_id, attrs={"loadable": False})
    collector = py_code.PyCodeCollector()
    collector.collect(CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=builder))
    atlas = builder.build()
    assert atlas.entities[file_id].attrs["loadable"] is False
    assert atlas.entities[file_id].fact("coded") is not None
