"""Tests for the misuse family: what one source file proves on its own.

Per rule: it fires on a source the readers settle, and it stays silent on the near miss that
is correct code. The source goes through the reader so the rule is tested on what the reader
states."""

from __future__ import annotations

import ast

from rosio import ids, model
from rosio.collect import cpp_code, py_code
from rosio.drift import misuse
from rosio.model import AtlasBuilder, Source

CPP_NODE = """
class Asker : public rclcpp::Node {
 public:
  Asker() : Node("asker") {
    client_ = create_client<Trigger>("trigger");
    sub_ = create_subscription<String>("in", 10, std::bind(&Asker::on_msg, this, _1));
  }
  void on_msg(const String::SharedPtr msg) {
    auto future = client_->async_send_request(std::make_shared<Trigger::Request>());
    auto reply = future.get();
    rclcpp::Time stamp(msg->header.stamp);
    if (rclcpp::Clock().now() - stamp > rclcpp::Duration(1, 0)) {}
  }
};
"""

PY_NODE = """
from rclpy.node import Node
from rclpy.time import Time


class Asker(Node):
    def __init__(self):
        super().__init__("asker")
        self.cli = self.create_client(Trigger, "trigger")
        self.timer = self.create_timer(1.0, self.on_tick)
        self.started = Time()

    def on_tick(self):
        reply = self.cli.call(Trigger.Request())
        if self.get_clock().now() - self.started > LIMIT:
            pass
"""


def _atlas(language: str, source: str) -> model.Atlas:
    builder = AtlasBuilder()
    file_id = ids.build_file("asker", f"src/asker.{'py' if language == 'python' else 'cpp'}")
    if language == "python":
        tree = ast.parse(source)
        values = {"blocking_waits": py_code.blocking_waits(tree)}
    else:
        src = source.encode()
        root = cpp_code.load_parser().parse(src).root_node
        values = {"blocking_waits": cpp_code.cpp_blocking_waits(root, src)}
    builder.node(file_id)
    builder.fact(
        file_id,
        model.Layer.CODED.value,
        values={key: rows for key, rows in values.items() if rows},
        confidence=model.Confidence.CERTAIN.value,
        evidence=[Source(file=file_id, locator="file")],
    )
    return builder.build()


def test_a_wait_on_a_default_group_client_inside_a_callback_is_reported() -> None:
    for language, source in (("cpp", CPP_NODE), ("python", PY_NODE)):
        found = misuse.callback_blocks_executor(_atlas(language, source))
        assert len(found) == 1, language
        assert found[0].severity == "error"
        assert found[0].where[0].line_hint == (10 if language == "cpp" else 14)
        assert found[0].id == misuse.callback_blocks_executor(_atlas(language, source))[0].id


def test_a_client_on_its_own_callback_group_is_not_reported() -> None:
    cpp = CPP_NODE.replace('("trigger")', '("trigger", rclcpp::ServicesQoS(), group_)')
    py = PY_NODE.replace('"trigger")', '"trigger", callback_group=self.group)')
    for language, source in (("cpp", cpp), ("python", py)):
        assert misuse.callback_blocks_executor(_atlas(language, source)) == [], language
