import pytest

from rosio.collect import cpp_code as cpp
from rosio.collect import py_code


def test_python_constructor_arguments_reach_endpoints_and_members():
    report = py_code.analyse_source(
        """
from rclpy.node import Node
class TestNode(Node):
    def __init__(self, remote, topic):
        super().__init__('client')
        self.topic = topic
        self.create_client(SetLogger, remote + '/set_logger_levels')
    def setup(self):
        self.create_publisher(Message, self.topic, 10)
def main():
    remote = 'LoggerServiceNode'
    topic = 'events'
    node = TestNode(remote, topic)
""",
        path="node.py",
        package="fixture",
        package_relative_path="node.py",
    )
    names = {(row["kind"], row["name"]) for row in report["endpoints"]}
    assert ("service_client", "LoggerServiceNode/set_logger_levels") in names
    assert ("publisher", "events") in names
    assert not report["unresolved"]


def test_python_different_constructor_values_are_not_chosen_arbitrarily():
    report = py_code.analyse_source(
        """
from rclpy.node import Node
class TestNode(Node):
    def __init__(self, topic):
        super().__init__('client')
        self.create_publisher(Message, topic, 10)
def main():
    first = TestNode('first')
    second = TestNode('second')
""",
        path="node.py",
        package="fixture",
        package_relative_path="node.py",
    )
    assert report["endpoints"][0]["name"] is None
    assert any(row["want"] == "topic_name" for row in report["unresolved"])


def test_python_branch_written_constructor_argument_stays_unknown():
    report = py_code.analyse_source(
        """
from rclpy.node import Node
class TestNode(Node):
    def __init__(self, topic):
        super().__init__('client')
        self.create_publisher(Message, topic, 10)
def main():
    topic = 'first'
    if enabled():
        topic = 'second'
    node = TestNode(topic)
""",
        path="node.py",
        package="fixture",
        package_relative_path="node.py",
    )
    assert report["endpoints"][0]["name"] is None


@pytest.mark.cpp
def test_cpp_constructor_arguments_reach_endpoints_and_members():
    unit = cpp.parse_text(
        """
class TestNode : public rclcpp::Node {
public:
  TestNode(const std::string & remote, const std::string & topic)
  : Node("client"), topic_(topic) {
    create_client<SetLogger>(remote + "/set_logger_levels");
  }
  void setup() { create_publisher<Message>(topic_, 10); }
  std::string topic_;
};
int main() {
  std::string remote = "LoggerServiceNode";
  std::string topic = "events";
  auto node = std::make_shared<TestNode>(remote, topic);
}
""",
        path="node.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    names = {(row["kind"], row["name"]) for row in report["endpoints"]}
    assert ("service_client", "LoggerServiceNode/set_logger_levels") in names
    assert ("publisher", "events") in names
    assert not report["unresolved"]


@pytest.mark.cpp
def test_cpp_readonly_construction_keeps_main_local_argument():
    unit = cpp.parse_text(
        """
class First : public rclcpp::Node {
public:
  First(const std::string & topic) : Node("first") {}
};
class Second : public rclcpp::Node {
public:
  Second(const std::string & topic) : Node("second") {
    create_publisher<Message>(topic, 10);
  }
};
int main() {
  std::string topic = "events";
  auto first = std::make_shared<First>(topic);
  auto second = std::make_shared<Second>(topic);
}
""",
        path="node.cpp",
    )
    cpp.analyse([unit])
    assert cpp.file_report(unit)["endpoints"][0]["name"] == "events"


@pytest.mark.cpp
def test_cpp_mutable_main_argument_is_not_assumed_unchanged_after_unknown_call():
    unit = cpp.parse_text(
        """
class TestNode : public rclcpp::Node {
public:
  TestNode(const std::string & topic) : Node("client") {
    create_publisher<Message>(topic, 10);
  }
};
int main() {
  std::string topic = "events";
  mutate(topic);
  auto node = std::make_shared<TestNode>(topic);
}
""",
        path="node.cpp",
    )
    cpp.analyse([unit])
    assert cpp.file_report(unit)["endpoints"][0]["name"] is None


@pytest.mark.cpp
def test_cpp_different_constructor_values_keep_both_contexts():
    unit = cpp.parse_text(
        """
class TestNode : public rclcpp::Node {
public:
  TestNode(const std::string & topic) : Node("client") {
    create_publisher<Message>(topic, 10);
  }
};
int main() {
  auto first = std::make_shared<TestNode>("first");
  auto second = std::make_shared<TestNode>("second");
}
""",
        path="node.cpp",
    )
    cpp.analyse([unit])
    report = cpp.file_report(unit)
    assert {row["name"] for row in report["endpoints"]} == {"first", "second"}
    assert len({row["construction"] for row in report["endpoints"]}) == 2


@pytest.mark.cpp
def test_cpp_one_blind_expression_in_two_symbols_stays_two_entries():
    unit = cpp.parse_text(
        """
class Talker : public rclcpp::Node {
public:
  Talker(const std::string & topic_name) : Node("talker"), topic_name_(topic_name) {
    create_publisher<Message>(topic_name_, 10);
  }
  std::string topic_name_;
};
class Listener : public rclcpp::Node {
public:
  Listener(const std::string & topic_name) : Node("listener"), topic_name_(topic_name) {
    create_subscription<Message>(topic_name_, 10, callback);
  }
  std::string topic_name_;
};
""",
        path="node.cpp",
    )
    cpp.analyse([unit])
    rows = cpp.file_report(unit)["unresolved"]
    owners = {row["owner_symbol"] for row in rows if row["want"] == "topic_name"}
    assert owners == {"Talker::Talker", "Listener::Listener"}


COMMON_NODES = """
from rclpy.node import Node
from std_msgs.msg import String


class Talker(Node):
    def __init__(self, topic_name, qos_profile, period=None):
        super().__init__('talker')
        self.create_publisher(String, topic_name, qos_profile)


class Listener(Node):
    def __init__(self, topic_name, qos_profile):
        super().__init__('listener')
        self.topic_name = topic_name

    def start_listening(self):
        self.create_subscription(String, self.topic_name, self.callback, 10)
"""

FIRST_MAIN = """
from px_ctx.common_nodes import Listener
from px_ctx.common_nodes import Talker


def main():
    topic = 'first_chatter'
    talker = Talker(topic, 10)
    listener = Listener(topic, qos_profile=10)
"""

SECOND_MAIN = """
import sys

from px_ctx.common_nodes import Listener
from px_ctx.common_nodes import Talker

LISTEN = 'second_listen'


def main():
    talker = Talker(sys.argv[1], 10)
    listener = Listener(LISTEN, 10)
"""


def _python_package(tmp_path, files):
    package = tmp_path / "src" / "px_ctx"
    (package / "px_ctx").mkdir(parents=True)
    (package / "package.xml").write_text('<package format="3"><name>px_ctx</name></package>')
    (package / "setup.py").write_text(
        "from setuptools import setup\n"
        "setup(name='px_ctx', entry_points={'console_scripts': "
        "['first = px_ctx.first:main', 'second = px_ctx.second:main']})\n"
    )
    for relative, text in files.items():
        (package / "px_ctx" / relative).write_text(text)
    out = py_code.analyse_workspace(tmp_path, "src")
    return next(row for row in out["files"] if row["path"].endswith("common_nodes.py"))


def test_python_constructions_in_other_modules_keep_one_context_each(tmp_path):
    record = _python_package(
        tmp_path,
        {"common_nodes.py": COMMON_NODES, "first.py": FIRST_MAIN, "second.py": SECOND_MAIN},
    )
    rows = {(row["kind"], row["name"]) for row in record["endpoints"]}
    assert rows == {
        ("publisher", "first_chatter"),
        ("publisher", None),
        ("subscriber", "first_chatter"),
        ("subscriber", "second_listen"),
    }
    assert len({row["construction"] for row in record["endpoints"]}) == 4
    assert {row["construction"] for row in record["nodes"]} == {
        row["construction"] for row in record["endpoints"]
    }
    blind = [(row["want"], row["expr"]) for row in record["unresolved"]]
    assert blind == [("topic_name", "topic_name")]


def test_python_class_built_outside_main_keeps_its_blind_spot(tmp_path):
    helper = FIRST_MAIN + "\n\ndef spare(name):\n    return Talker(name, 10)\n"
    record = _python_package(tmp_path, {"common_nodes.py": COMMON_NODES, "first.py": helper})
    talker = [row for row in record["endpoints"] if row["kind"] == "publisher"]
    assert [(row["name"], row.get("construction")) for row in talker] == [(None, None)]
    listener = [row for row in record["endpoints"] if row["kind"] == "subscriber"]
    assert [row["name"] for row in listener] == ["first_chatter"]
