"""Cases from the ninth first run trial and the review that followed the eighth."""

from __future__ import annotations

from pathlib import Path

from rosio import ids
from tests.test_eighth_trial import LOOKUP, STATIC, findings, node, python_workspace

UNREAD = 'Node(package="robot_localization", executable="ekf_node", name="ekf")'


# ---------------------------------------------------------------------------
# a node rosio could not read may broadcast the frame a lookup wants


def test_a_frame_an_unread_node_could_broadcast_has_no_publisher_claim(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {"planner": node("planner", LOOKUP.format("base_footprint"))},
        STATIC.format("base_to_laser", "base_footprint", "laser"),
        UNREAD,
    )
    assert findings(atlas, "frame.no_publisher") == []


def test_a_frame_nothing_could_broadcast_still_has_no_publisher(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {"planner": node("planner", LOOKUP.format("base_footprint"))},
        STATIC.format("base_to_laser", "base_footprint", "laser"),
    )
    found = findings(atlas, "frame.no_publisher")
    assert [item.subject for item in found] == [ids.build_frame("base_footprint")]


# ---------------------------------------------------------------------------
# two robots that remap /tf into their own namespace hold two trees

ROBOT_STATIC = (
    'Node(package="tf2_ros", executable="static_transform_publisher", name="laser_tf", '
    'namespace="{0}", remappings=[("/tf", "tf"), ("/tf_static", "tf_static")], '
    'arguments=["--frame-id", "base_link", "--child-frame-id", "laser"])'
)


def test_one_frame_in_two_namespaced_tf_trees_is_not_broadcast_twice(tmp_path: Path):
    atlas = python_workspace(
        tmp_path, {}, ROBOT_STATIC.format("robot1"), ROBOT_STATIC.format("robot2")
    )
    assert findings(atlas, "frame.broadcast_twice") == []


# ---------------------------------------------------------------------------
# a probe files its observation against the profile whose launch command runs


def test_a_probe_files_against_the_profile_whose_launch_command_runs(monkeypatch):
    from rosio import cli
    from rosio.collect import proc
    from rosio.config import Entrypoint, ProfileSpec

    def spec(slug, sim):
        entry = Entrypoint("launch", "arm_bringup", "arm.launch.py", (("use_sim", sim),))
        return ProfileSpec(slug=slug, entrypoints=(entry,))

    running = proc.RosProcess(
        argv=("ros2", "launch", "arm_bringup", "arm.launch.py", "use_sim:=true"),
        verb="launch",
        target="arm_bringup arm.launch.py",
    )
    monkeypatch.setattr(proc, "available", lambda root=proc.PROC: True)
    monkeypatch.setattr(proc, "scan", lambda root=proc.PROC, domain=None: [running])
    (chosen,) = cli._running_profiles((spec("real", "false"), spec("sim", "true")))
    assert chosen.slug == "sim"


# ---------------------------------------------------------------------------
# a get_parameter of a name the node never declares raises at that line


def test_a_read_of_an_undeclared_parameter_is_reported_and_a_read_with_default_is_not(
    tmp_path: Path,
):
    body = (
        "        self.declare_parameter('timeout', 1.0)\n"
        "        self.get_parameter('timeout').value\n"
        "        self.get_parameter('watchdog').value\n"
        "        self.get_parameter_or('limit', 2.0)\n"
        "        if self.has_parameter('gain'):\n"
        "            self.get_parameter('gain').value\n"
    )
    atlas = python_workspace(tmp_path, {"sup": node("sup", body)})
    found = findings(atlas, "param.read_undeclared")
    assert [item.subject for item in found] == [ids.build_param("node:/sup", "watchdog")]


# ---------------------------------------------------------------------------
# a message stamped with a frame the model names another way


def test_a_frame_written_into_a_header_that_no_model_has_is_reported(tmp_path: Path):
    model_text = (
        '<robot name="r"><link name="base_link"/><link name="camera_link"/>'
        '<joint name="j" type="fixed"><parent link="base_link"/><child link="camera_link"/>'
        "</joint></robot>"
    )
    (tmp_path / "src" / "d" / "urdf").mkdir(parents=True)
    (tmp_path / "src" / "d" / "package.xml").write_text(
        "<package><name>d</name></package>", encoding="utf-8"
    )
    (tmp_path / "src" / "d" / "urdf" / "r.urdf").write_text(model_text, encoding="utf-8")
    body = "        m = Image()\n        m.header.frame_id = 'camera_frame'\n"
    atlas = python_workspace(tmp_path, {"cam": node("cam", body)})
    found = [
        item for item in findings(atlas, "frame.not_in_urdf") if "stamps messages" in item.title
    ]
    assert [item.subject for item in found] == ["file:p|p/cam.py"]
    assert "camera_link" in found[0].say


# ---------------------------------------------------------------------------
# a path does not run through a subscription the publisher's QoS never matches


def test_a_path_does_not_cross_a_qos_refused_subscription(tmp_path: Path):
    from rosio import query

    head = "from rclpy.qos import QoSProfile, qos_profile_sensor_data\n"
    atlas = python_workspace(
        tmp_path,
        {
            "cam": head
            + node("cam", "        self.create_publisher(Image, 'pts', qos_profile_sensor_data)\n"),
            "mon": head
            + node(
                "mon",
                "        self.create_subscription(Image, 'pts', self.cb, QoSProfile(depth=5))\n",
            ),
        },
    )
    for start, end in (("/cam", "/mon"), ("/mon", "/cam")):
        answer = query.path(atlas, start, end)
        assert answer.data["qos_cut"] is True
        assert "publishes" not in {hop["kind"] for hop in answer.data["hops"]}


# ---------------------------------------------------------------------------
# a publisher a timer callback drives states the timer's period


def test_a_publisher_driven_by_a_timer_states_its_period(tmp_path: Path):
    body = (
        "        rate = self.declare_parameter('rate', 10.0).value\n"
        "        self.pub = self.create_publisher(Twist, 'out', 10)\n"
        "        self.create_timer(1.0 / rate, self.tick)\n"
        "    def tick(self):\n"
        "        self.pub.publish(Twist())\n"
    )
    atlas = python_workspace(tmp_path, {"beat": node("beat", body)})
    periods = {
        fact.values.get("period_s")
        for edge in atlas.edges.values()
        if edge.kind == "publishes" and edge.dst == "topic:/out"
        for fact in edge.facts
    }
    assert periods == {0.1}
