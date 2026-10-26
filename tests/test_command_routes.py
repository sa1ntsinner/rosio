"""Command substitutions stay unknown across launch records and atlas facts."""

from pathlib import Path

import pytest

from rosio import cli, model, schema


def _extract_launch(tmp_path: Path, body: str, child: str | None = None) -> model.Atlas:
    package = tmp_path / "src" / "demo"
    package.mkdir(parents=True)
    (package / "package.xml").write_text(
        '<package format="3"><name>demo</name><version>0.0.0</version>'
        '<description>test</description><maintainer email="test@example.com">Test</maintainer>'
        "<license>Apache-2.0</license></package>"
    )
    (package / "params.yaml").write_text("talker:\n  ros__parameters:\n    use_sim_time: false\n")
    if child is not None:
        (package / "child.launch.py").write_text(child)
    (package / "test.launch.py").write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, "
        "OpaqueFunction, TimerAction, RegisterEventHandler\n"
        "from launch.event_handlers import OnProcessExit\n"
        "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
        "from launch.conditions import IfCondition, LaunchConfigurationEquals\n"
        "from launch.substitutions import Command, LaunchConfiguration\n"
        "from launch_ros.actions import Node\n"
        "from nav2_common.launch import RewrittenYaml\n"
        "from pathlib import Path\n"
        "def generate_launch_description():\n"
        "    p = str(Path(__file__).with_name('params.yaml'))\n"
        "    command = Command(['printf true'])\n"
        "    return LaunchDescription(" + body + ")\n"
    )
    config = tmp_path / "rosio.yaml"
    config.write_text(
        "roots:\n  - path: src\n    kind: src\nprofiles:\n  - slug: test\n"
        "    entrypoints:\n      - launch demo test.launch.py\n"
    )
    out = tmp_path / "out"
    assert cli.main(
        ["extract", str(tmp_path), "--config", str(config), "--out", str(out), "--quiet"]
    ) in (0, 1)
    atlas = model.Atlas.loads((out / "atlas.json").read_text())
    assert not schema.validate(atlas.to_json())
    return atlas


@pytest.mark.ros
@pytest.mark.parametrize(
    "route", ["rewrite", "dotted", "root", "root_ns", "condition", "indirect", "equals", "group"]
)
def test_commands_in_wrappers_and_conditions_stay_unknown(tmp_path: Path, route):
    arguments = {
        "rewrite": "parameters=[RewrittenYaml(source_file=p, "
        "param_rewrites={'use_sim_time': command}, convert_types=True)]",
        "dotted": "parameters=[RewrittenYaml(source_file=p, "
        "param_rewrites={'talker.ros__parameters.use_sim_time': command}, convert_types=True)]",
        "root": "parameters=[RewrittenYaml(source_file=p, param_rewrites={}, root_key=command)]",
        "root_ns": "namespace='/robot', parameters=[RewrittenYaml(source_file=p, "
        "param_rewrites={}, root_key=command)]",
        "condition": "condition=IfCondition(command)",
        "indirect": "condition=IfCondition(LaunchConfiguration('enabled'))",
        "equals": "condition=LaunchConfigurationEquals('enabled', 'true')",
        "group": "",
    }
    node = (
        "Node(package='demo_nodes_cpp', executable='talker', name='talker', "
        + arguments[route]
        + ")"
    )
    body = "[" + node + "]"
    if route in ("indirect", "equals"):
        body = "[DeclareLaunchArgument('enabled', default_value=command), " + node + "]"
    elif route == "group":
        body = "[GroupAction(actions=[" + node + "], condition=IfCondition(command))]"
    atlas = _extract_launch(tmp_path, body)
    blinds = [u for u in atlas.unresolved if u.reason == "command_substitution"]
    assert blinds, [(u.reason, u.expr) for u in atlas.unresolved]
    assert "__rosio_command_" not in atlas.canonical_bytes().decode()
    assert "~uncarried" not in atlas.canonical_bytes().decode()
    if route not in ("rewrite", "dotted", "root", "root_ns"):
        assert any(u.owner.startswith("launchent:") and u.role == "condition" for u in blinds)
        assert atlas.counters["resolution_rate"]["other"] < 1.0
        launches = [e for e in atlas.entities.values() if e.kind == "launchent"]
        assert any(
            g["outcome"] == "unknown"
            for e in launches
            for f in e.facts
            for g in f.values.get("gates", [])
        )
    else:
        owner = "/robot/talker" if route == "root_ns" else "/talker"
        parameter = atlas.entities[f"param:{owner}|use_sim_time"]
        assert parameter.state == "other"
        assert not parameter.effective
        assert parameter.candidates and not any(c.wins for c in parameter.candidates)
        assert parameter.candidates[0].value is None


@pytest.mark.ros
@pytest.mark.parametrize("second_site", [False, True])
def test_mixed_clock_uses_reachable_launch_sites(tmp_path, second_site):
    nodes = [
        "Node(package='demo_nodes_cpp', executable='talker', name='running', "
        "parameters=[{'use_sim_time': True}])",
        "Node(package='demo_nodes_cpp', executable='listener', name='disabled', "
        "condition=IfCondition('false'), parameters=[{'use_sim_time': False}])",
    ]
    if second_site:
        nodes.append(
            "Node(package='demo_nodes_cpp', executable='listener', name='disabled', "
            "condition=IfCondition('true'), parameters=[{'use_sim_time': False}])"
        )
    atlas = _extract_launch(tmp_path, "[" + ", ".join(nodes) + "]")
    assert bool([f for f in atlas.findings if f.rule == "param.sim_time_mixed"]) == second_site


@pytest.mark.parametrize(
    "value",
    [
        {"nested": ["__rosio_command_74727565__"]},
        {"nested": [{"~uncarried": {"expr": "true", "reason": "command_substitution"}}]},
    ],
)
def test_internal_substitution_markers_are_refused_in_facts(value):
    from rosio.errors import ContractError

    with pytest.raises(ContractError, match="substitution marker"):
        model.LayerFact(layer="declared", values=value)


@pytest.mark.ros
def test_a_command_passed_into_an_include_stays_unknown(tmp_path):
    atlas = _extract_launch(
        tmp_path,
        "[IncludeLaunchDescription(PythonLaunchDescriptionSource("
        "str(Path(__file__).with_name('child.launch.py'))), "
        "launch_arguments={'value': command}.items())]",
        "from launch import LaunchDescription\n"
        "from launch.substitutions import LaunchConfiguration\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', executable='talker', "
        "name='talker', parameters=[{'unknown': LaunchConfiguration('value')}])])\n",
    )
    assert atlas.counters["passes"]["complete"]
    assert not [
        e.attrs.get("exceptions")
        for e in atlas.entities.values()
        if e.kind == "run" and e.attrs.get("exceptions")
    ]
    parameter = atlas.entities["param:/talker|unknown"]
    assert parameter.state == "other"
    assert not parameter.effective
    assert any(u.reason == "command_substitution" for u in atlas.unresolved)
    assert "__rosio_command_" not in atlas.canonical_bytes().decode()


@pytest.mark.ros
@pytest.mark.parametrize(
    "action", ["opaque", "timer_condition", "timer_period", "include", "event"]
)
def test_commands_in_action_gates_keep_the_node_and_blind_spot(tmp_path, action):
    node = "Node(package='demo_nodes_cpp', executable='talker', name='talker')"
    body = {
        "opaque": "[OpaqueFunction(function=lambda context: ["
        + node
        + "], condition=IfCondition(command))]",
        "timer_condition": "[TimerAction(period=1.0, actions=["
        + node
        + "], condition=IfCondition(command))]",
        "timer_period": "[TimerAction(period=command, actions=[" + node + "])]",
        "include": "[IncludeLaunchDescription(PythonLaunchDescriptionSource("
        "str(Path(__file__).with_name('child.launch.py'))), condition=IfCondition(command))]",
        "event": "[RegisterEventHandler(OnProcessExit(on_exit=["
        + node
        + "]), condition=IfCondition(command))]",
    }[action]
    child = (
        "from launch import LaunchDescription\nfrom launch_ros.actions import Node\n"
        "def generate_launch_description():\n    return LaunchDescription([" + node + "])\n"
    )
    atlas = _extract_launch(tmp_path, body, child if action == "include" else None)
    assert "node:/talker" in atlas.entities
    assert any(u.reason == "command_substitution" for u in atlas.unresolved)
    assert not [
        e.attrs["exceptions"]
        for e in atlas.entities.values()
        if e.kind == "run" and e.attrs.get("exceptions")
    ]
    assert "__rosio_command_" not in atlas.canonical_bytes().decode()
