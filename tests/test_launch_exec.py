"""The sandboxed launch resolver, against every fixture in ws_launch.

Every executed case is compared with the hand written answer in `expected/launch`. Three
fields are deliberately not compared, and the reason is in `expected/README.md`: `note` and
`case` are prose about the fixture, the `conditions` strings are advisory text, and the
`predicates` keys are advisory too, since a short circuited `and` never reaches the probe at
all. What the predicates must get right is asserted on its own, per case.

One projection is applied before comparing: a node whose condition evaluated false is kept by
the resolver, marked unreachable, and left out of the comparison. The expected files record
the reachable set only. Keeping the node is deliberate, so the fact that a launch file can
start it survives, and `test_false_condition_node_is_kept` pins that behaviour.
"""

from __future__ import annotations

import json
import shutil
from pathlib import Path

import pytest

from rosio.resolve import launch_exec, records, ros_args, substitutions

WS = "ws_launch"

ROS_SETUP = ("/opt/ros/humble/setup.bash",)


@pytest.mark.ros
@pytest.mark.parametrize("value_type", ["str", "int", "None"])
def test_a_command_is_a_blind_value_and_never_runs(tmp_path: Path, value_type: str) -> None:
    from rosio.resolve._launch_child import uncarried_reason

    mark = tmp_path / "started"
    entry = tmp_path / "command.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import Command, LaunchConfiguration\n"
        "from launch_ros.actions import Node\n"
        "from launch_ros.parameter_descriptions import ParameterValue\n"
        "def generate_launch_description():\n"
        "    command = Command(['touch ', LaunchConfiguration('mark')])\n"
        f"    value = ParameterValue(command, value_type={value_type})\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', executable='talker',\n"
        "        name='talker', parameters=[{'unknown': value, 'known': 42}, {'later': True}],\n"
        "        remappings=[('chatter', '/out')])])\n"
    )
    walk = launch_exec.walk(str(entry), args=[("mark", str(mark))])
    assert not mark.exists()
    assert not walk.exceptions
    assert len(walk.nodes) == 1
    node = walk.nodes[0]
    values = dict(node.params[0].values)
    assert values["known"] == 42
    assert dict(node.params[1].values) == {"later": True}
    assert uncarried_reason(values["unknown"]) == (f"touch {mark}", "command_substitution")
    assert [(r.frm, r.to) for r in node.remaps] == [("chatter", "/out")]
    assert not node.file_reads
    assert str(tmp_path) not in " ".join(node.role_content)


@pytest.mark.ros
def test_a_launch_exception_is_still_recorded(tmp_path: Path) -> None:
    entry = tmp_path / "raises.launch.py"
    entry.write_text(
        "def generate_launch_description():\n    raise RuntimeError('broken launch')\n"
    )
    walk = launch_exec.walk(str(entry))
    assert [(e.phase, e.exc_type, e.message) for e in walk.exceptions] == [
        ("describe", "RuntimeError", "broken launch")
    ]


@pytest.mark.ros
@pytest.mark.parametrize("field", ["name", "namespace", "arguments", "remappings", "parameters"])
def test_a_command_in_a_node_field_never_becomes_a_literal(tmp_path: Path, field: str) -> None:
    from rosio import model
    from rosio.resolve import identity

    values = {
        "name": "command",
        "namespace": "command",
        "arguments": "[command]",
        "remappings": "[('chatter', command)]",
        "parameters": "[command]",
    }
    entry = tmp_path / "command.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import Command\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    command = Command(['printf result'])\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', executable='talker', "
        f"{field}={values[field]})])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert not walk.exceptions
    assert len(walk.nodes) == 1
    node = walk.nodes[0]
    assert len(node.command_blinds) == 1
    assert node.command_blinds[0][1] == "printf result"
    assert "__rosio_command_" not in model.dumps(launch_exec.encode_walk(walk))
    if field in ("name", "namespace"):
        assert identity.identify(node).fqn is None
    if field == "name":
        assert identity.identify(node).name_source == "unknown"


@pytest.mark.ros
def test_group_scoped_rules_cite_the_action_that_pushed_them(tmp_path: Path) -> None:
    """A SetRemap above an include belongs to the file that writes it, not to the node."""
    child = tmp_path / "child.launch.py"
    child.write_text(
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', executable='talker',\n"
        "        name='talker', remappings=[('chatter', '/out')])])\n"
    )
    params = tmp_path / "params.yaml"
    params.write_text("/**:\n  ros__parameters:\n    rate: 5\n")
    parent = tmp_path / "parent.launch.py"
    parent.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import GroupAction, IncludeLaunchDescription\n"
        "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
        "from launch_ros.actions import SetParameter, SetParametersFromFile, SetRemap\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([GroupAction([\n"
        "        SetRemap('/map', 'map'),\n"
        "        SetParameter(name='use_sim_time', value=True),\n"
        f"        SetParametersFromFile({str(params)!r}),\n"
        f"        IncludeLaunchDescription(PythonLaunchDescriptionSource({str(child)!r})),\n"
        "    ])])\n"
    )
    walks = [launch_exec.walk(str(parent)) for _ in range(2)]
    assert walks[0].nodes == walks[1].nodes
    walk = walks[0]
    assert not walk.exceptions
    [node] = walk.nodes
    assert node.site.path == str(child)
    sites = {(r.frm, r.to): r.site for r in node.remaps}
    assert sites[("/map", "map")] == records.Site(str(parent), "SetRemap", 7)
    assert sites[("chatter", "/out")].path == str(child)
    assert [(p.site.path, p.site.locator, p.site.line_hint) for p in node.params] == [
        (str(parent), "SetParameter", 8),
        (str(parent), "SetParametersFromFile", 9),
    ]


def test_command_paths_in_options_and_quotes_are_stated_without_machine_prefixes():
    text = (
        "xacro '/workspace/robot model.xacro' --input=/workspace/view.rviz "
        "mesh:=/workspace/mesh.stl"
    )
    assert substitutions.command_text(
        text, lambda path: "file:example|" + path.rsplit("/", 1)[-1]
    ) == (
        "xacro 'file:example|robot model.xacro' --input=file:example|view.rviz "
        "mesh:=file:example|mesh.stl"
    )


@pytest.mark.parametrize("value", ["__rosio_command_f__", "__rosio_command_ff__"])
def test_a_literal_that_resembles_a_command_marker_is_preserved(value):
    from rosio.resolve._launch_child import _plain

    assert _plain(value) == value


@pytest.mark.ros
def test_an_unknown_xacro_path_never_becomes_a_file(tmp_path: Path):
    entry = tmp_path / "missing.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import Command, PathJoinSubstitution\n"
        "from launch_ros.substitutions import FindPackageShare\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    path = PathJoinSubstitution([FindPackageShare('not_a_real_package'), 'robot.xacro'])\n"
        "    return LaunchDescription([Node(package='robot_state_publisher', "
        "executable='robot_state_publisher', "
        "parameters=[{'robot_description': Command(['xacro ', path])}])])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert not walk.exceptions
    assert not walk.nodes[0].file_reads


def _xacro_walk(tmp_path: Path, namespace: str, args=()) -> records.LaunchWalk:
    model = tmp_path / "robot.urdf"
    model.write_text("<robot name='bot'/>", encoding="utf-8")
    entry = tmp_path / "model.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import Command, LaunchConfiguration\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        f"    ns = {namespace}\n"
        "    return LaunchDescription([Node(package='robot_state_publisher', "
        "executable='robot_state_publisher', parameters=[{'robot_description': "
        f"Command(['xacro ', {str(model)!r}, ' namespace:=', ns, ' use_sim:=false'])}}])])\n"
    )
    return launch_exec.walk(str(entry), args=args)


@pytest.mark.ros
def test_a_model_command_carries_its_xacro_arguments(tmp_path: Path):
    walk = _xacro_walk(tmp_path, "LaunchConfiguration('namespace')", [("namespace", "tb3/")])
    assert not walk.exceptions
    (read,) = walk.nodes[0].file_reads
    assert read.role == "robot_model"
    assert read.xacro_args == (("namespace", "tb3/"), ("use_sim", "false"))
    assert read.blind == ""


@pytest.mark.ros
def test_a_model_argument_the_walk_cannot_perform_is_blind_and_names_nothing(tmp_path: Path):
    walk = _xacro_walk(tmp_path, "LaunchConfiguration('never_declared')")
    (read,) = walk.nodes[0].file_reads
    assert read.role == "robot_model"
    assert read.xacro_args == ()
    assert "never_declared" in read.blind


@pytest.mark.ros
def test_an_unset_environment_variable_is_named(tmp_path: Path):
    entry = tmp_path / "env.launch.py"
    entry.write_text(
        "import os\n"
        "from launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    os.environ['ROSIO_TEST_MODEL']\n"
        "    return LaunchDescription([])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert [(e.exc_type, e.env) for e in walk.exceptions] == [("KeyError", "ROSIO_TEST_MODEL")]
    stated = launch_exec.walk(str(entry), env=[("ROSIO_TEST_MODEL", "burger")])
    assert not stated.exceptions


@pytest.mark.ros
def test_a_variable_read_at_import_leaves_a_launch_file_loadable(tmp_path: Path):
    """turtlebot3's navigation2.launch.py reads its model at module level. The file is a
    launch file the profile has not configured, not a file of another kind."""
    entry = tmp_path / "import_env.launch.py"
    entry.write_text(
        "import os\n"
        "MODEL = os.environ['ROSIO_TEST_MODEL']\n"
        "def generate_launch_description():\n"
        "    from launch import LaunchDescription\n"
        "    return LaunchDescription([])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert [(e.phase, e.env) for e in walk.exceptions] == [("import", "ROSIO_TEST_MODEL")]
    assert walk.loadable
    missing = tmp_path / "import_error.launch.py"
    missing.write_text("import not_a_module_anywhere\n")
    walk = launch_exec.walk(str(missing))
    # A launch file that imports a module this machine lacks is still a launch file.
    assert walk.loadable
    assert [e.exc_type for e in walk.exceptions] == ["ModuleNotFoundError"]
    broken = tmp_path / "syntax_error.launch.py"
    broken.write_text("def generate_launch_description(:\n")
    assert not launch_exec.walk(str(broken)).loadable


MISSING_DRIVER = """\
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    lidar = os.path.join(get_package_share_directory("rosio_no_such_driver"), "launch")
    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource([lidar, "/driver.launch.py"])),
        Node(package="lx", executable="base", name="base"),
    ])
"""


@pytest.mark.ros
def test_a_missing_driver_package_costs_its_include_and_not_the_profile(tmp_path: Path):
    """turtlebot3's robot.launch.py looks up the lidar driver while it builds its description,
    and one missing driver emptied the whole robot profile."""
    entry = tmp_path / "robot.launch.py"
    entry.write_text(MISSING_DRIVER, encoding="utf-8")
    walk = launch_exec.walk(str(entry))
    assert [(node.pkg, node.name) for node in walk.nodes] == [("lx", "base")]
    assert [(e.phase, e.exc_type) for e in walk.exceptions] == [
        ("describe", "PackageNotFoundError")
    ]
    assert "rosio_no_such_driver" in walk.exceptions[0].message
    (include,) = walk.includes
    assert not include.resolved
    assert include.reason == launch_exec.PACKAGE_NOT_FOUND
    assert include.path == "$(find-pkg-share rosio_no_such_driver)/launch/driver.launch.py"


OPTIONAL_DRIVER = """\
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    nodes = [Node(package="lx", executable="base", name="base")]
    try:
        get_package_share_directory("rosio_no_such_driver")
        nodes.append(Node(package="rosio_no_such_driver", executable="drv", name="drv"))
    except PackageNotFoundError:
        pass
    return LaunchDescription(nodes)
"""


@pytest.mark.ros
def test_a_launch_file_that_handles_a_missing_package_builds_as_ros2_launch_builds_it(
    tmp_path: Path,
):
    """A probe for an optional package is the file's own decision: the guarded node is not
    started and nothing raised."""
    entry = tmp_path / "optional.launch.py"
    entry.write_text(OPTIONAL_DRIVER, encoding="utf-8")
    walk = launch_exec.walk(str(entry))
    assert [(node.pkg, node.name) for node in walk.nodes] == [("lx", "base")]
    assert walk.exceptions == []


@pytest.mark.ros
def test_a_description_that_raises_for_another_reason_still_says_so(tmp_path: Path):
    entry = tmp_path / "broken.launch.py"
    entry.write_text(
        "def generate_launch_description():\n    raise RuntimeError('boom')\n", encoding="utf-8"
    )
    walk = launch_exec.walk(str(entry))
    assert [(e.phase, e.exc_type) for e in walk.exceptions] == [("describe", "RuntimeError")]
    assert walk.nodes == []


@pytest.mark.ros
def test_a_dictionary_key_error_is_not_an_environment_variable(tmp_path: Path):
    entry = tmp_path / "dict.launch.py"
    entry.write_text("def generate_launch_description():\n    return {}['ROSIO_TEST_MODEL']\n")
    walk = launch_exec.walk(str(entry))
    assert [(e.exc_type, e.env) for e in walk.exceptions] == [("KeyError", "")]


URDF = '<?xml version="1.0"?>\n<robot name="bot">\n  <link name="base_link"/>\n</robot>\n'


def _model_walk(tmp_path: Path, read: str) -> records.LaunchWalk:
    model = tmp_path / "robot.urdf"
    model.write_text(URDF)
    (tmp_path / "copy.urdf").write_text(URDF)
    entry = tmp_path / "model.launch.py"
    entry.write_text(
        "from pathlib import Path\n"
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        f"    path = {str(model)!r}\n"
        f"    text = {read}\n"
        "    return LaunchDescription([Node(package='robot_state_publisher', "
        "executable='robot_state_publisher', "
        "parameters=[{'robot_description': text}])])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert not walk.exceptions
    return walk


@pytest.mark.ros
@pytest.mark.parametrize(
    "read",
    [
        "open(path).read()",
        "Path(path).read_text()",
        "''.join(open(path, 'rt', encoding='utf-8').readlines())",
    ],
)
def test_a_model_read_whole_in_the_launch_file_is_a_robot_model_read(tmp_path: Path, read: str):
    walk = _model_walk(tmp_path, read)
    assert [(r.path, r.role) for r in walk.nodes[0].file_reads] == [
        (str(tmp_path / "robot.urdf"), "robot_model")
    ]
    assert walk.nodes[0].file_reads[0].site == walk.nodes[0].site


@pytest.mark.ros
@pytest.mark.parametrize(
    "read",
    [
        "open(path).read().replace('bot', 'other')",
        "open(path).read()[:-1]",
        "open(path).read() if open(path.replace('robot', 'copy')).read() else ''",
        "open(path, 'rb').read().decode()",
    ],
)
def test_a_model_text_that_is_not_one_files_text_is_not_attributed(tmp_path: Path, read: str):
    walk = _model_walk(tmp_path, read)
    assert not walk.nodes[0].file_reads


@pytest.mark.ros
@pytest.mark.parametrize("serialise", ["toxml()", "toprettyxml(indent='  ')"])
def test_a_document_xacro_processed_is_a_robot_model_read(tmp_path: Path, serialise: str):
    pytest.importorskip("xacro")
    model = tmp_path / "robot.urdf.xacro"
    model.write_text(
        '<?xml version="1.0"?>\n<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro">\n'
        '  <xacro:property name="base" value="base_link"/>\n  <link name="${base}"/>\n</robot>\n'
    )
    entry = tmp_path / "xacro.launch.py"
    entry.write_text(
        "import xacro\n"
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        f"    text = xacro.process_file({str(model)!r}).{serialise}\n"
        "    return LaunchDescription([Node(package='robot_state_publisher', "
        "executable='robot_state_publisher', "
        "parameters=[{'robot_description': text}])])\n"
    )
    first = launch_exec.walk(str(entry))
    second = launch_exec.walk(str(entry))
    assert not first.exceptions
    assert [(r.path, r.role) for r in first.nodes[0].file_reads] == [(str(model), "robot_model")]
    assert first.nodes[0].file_reads == second.nodes[0].file_reads


PY_XACRO_MODEL = (
    '<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro">'
    '<xacro:arg name="x" default="0.1"/><xacro:arg name="prefix" default=""/>'
    '<link name="$(arg prefix)base_link"/><link name="$(arg prefix)base_scan"/>'
    '<joint name="scan_joint" type="fixed"><parent link="$(arg prefix)base_link"/>'
    '<child link="$(arg prefix)base_scan"/><origin xyz="$(arg x) 0 0.2"/></joint></robot>'
)


def _py_xacro_walk(tmp_path: Path, call: str, value: str, args=()) -> records.LaunchWalk:
    model = tmp_path / "robot.urdf.xacro"
    model.write_text(PY_XACRO_MODEL, encoding="utf-8")
    entry = tmp_path / "py_xacro.launch.py"
    entry.write_text(
        "import xacro\n"
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument, OpaqueFunction\n"
        "from launch.substitutions import Command, LaunchConfiguration\n"
        "from launch_ros.actions import Node\n"
        "def make(context):\n"
        f"    path = {str(model)!r}\n"
        f"    mappings = {{'x': {value}, 'prefix': 'tb3/'}}\n"
        f"    text = {call}\n"
        "    return [Node(package='robot_state_publisher', executable='robot_state_publisher', "
        "parameters=[{'robot_description': text}])]\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([DeclareLaunchArgument('x', default_value='0.1'), "
        "OpaqueFunction(function=make)])\n"
    )
    return launch_exec.walk(str(entry), args=args)


PY_XACRO_CALLS = [
    "xacro.process_file(path, mappings=mappings).toxml()",
    "xacro.process(path, mappings=mappings)",
    "xacro.process_doc(doc := xacro.parse(open(path)), mappings=mappings) or doc.toxml()",
]


@pytest.mark.ros
@pytest.mark.parametrize("call", PY_XACRO_CALLS)
def test_a_model_xacro_expands_in_python_carries_its_mappings(tmp_path: Path, call: str):
    pytest.importorskip("xacro")
    value = "LaunchConfiguration('x').perform(context)"
    walk = _py_xacro_walk(tmp_path, call, value, [("x", "0.3")])
    assert not walk.exceptions
    (read,) = walk.nodes[0].file_reads
    assert (read.path, read.role) == (str(tmp_path / "robot.urdf.xacro"), "robot_model")
    assert read.xacro_args == (("prefix", "tb3/"), ("x", "0.3"))
    assert read.blind == ""


@pytest.mark.ros
def test_a_python_mapping_the_walk_cannot_perform_is_blind_and_names_nothing(tmp_path: Path):
    pytest.importorskip("xacro")
    value = "context.perform_substitution(Command('hostname'))"
    walk = _py_xacro_walk(tmp_path, PY_XACRO_CALLS[0], value)
    assert not walk.exceptions
    (read,) = walk.nodes[0].file_reads
    assert read.role == "robot_model"
    assert read.xacro_args == ()
    assert read.blind == f"xacro {tmp_path / 'robot.urdf.xacro'} prefix:=tb3/ x:=Command(hostname)"


def _installed_walk(tmp_path: Path, read: str) -> records.LaunchWalk:
    """A launch file where a binary install puts it, beside a library that can read too."""
    model = tmp_path / "robot.urdf"
    model.write_text(URDF)
    launch_dir = tmp_path / "opt" / "ros" / "x" / "share" / "bot" / "launch"
    library = tmp_path / "opt" / "ros" / "x" / "lib" / "python3.12" / "site-packages"
    launch_dir.mkdir(parents=True)
    library.mkdir(parents=True)
    (library / "bot_helper.py").write_text(
        "import os\n"
        "def model(path):\n"
        "    os.path.exists(path + '.library')\n"
        "    return open(path).read()\n"
    )
    entry = launch_dir / "bot.launch.py"
    entry.write_text(
        "import os, sys\n"
        f"sys.path.insert(0, {str(library)!r})\n"
        "import bot_helper\n"
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        f"    path = {str(model)!r}\n"
        "    os.path.exists(path + '.launch')\n"
        f"    text = {read}\n"
        "    return LaunchDescription([Node(package='robot_state_publisher', "
        "executable='robot_state_publisher', "
        "parameters=[{'robot_description': text}])])\n"
    )
    walk = launch_exec.walk(str(entry), launch_sandbox="off")
    assert not walk.exceptions
    return walk


@pytest.mark.ros
def test_an_installed_launch_file_records_its_model_read(tmp_path: Path):
    walk = _installed_walk(tmp_path, "open(path).read()")
    assert [(r.path, r.role) for r in walk.nodes[0].file_reads] == [
        (str(tmp_path / "robot.urdf"), "robot_model")
    ]
    probes = " ".join(walk.predicates)
    assert "robot.urdf.launch" in probes
    assert "robot.urdf.library" not in probes


@pytest.mark.ros
def test_a_read_by_library_code_is_not_a_launch_file_read(tmp_path: Path):
    walk = _installed_walk(tmp_path, "bot_helper.model(path)")
    assert not walk.nodes[0].file_reads
    assert "robot.urdf.library" not in " ".join(walk.predicates)


@pytest.mark.ros
def test_a_model_read_is_the_same_on_every_walk(tmp_path: Path):
    reads = {_model_walk(tmp_path, "open(path).read()").nodes[0].file_reads for _ in range(2)}
    assert len(reads) == 1


@pytest.mark.ros
@pytest.mark.parametrize("executable", ["xacro", "/usr/bin/xacro"])
def test_file_reads_keep_the_node_site_and_argument_roles(tmp_path: Path, executable: str):
    import shlex

    from rosio.resolve._launch_child import _file_reads, uncarried

    model = str(tmp_path / "robot.xacro")
    rviz = str(tmp_path / "view.rviz")
    params = str(tmp_path / "params.yaml")
    other = str(tmp_path / "data.txt")
    site = records.Site("launch/robot.py", "Node(robot)", 12)
    node = records.ResolvedNode(
        pkg="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="/",
        params=(
            records.ParamSource(
                order=0,
                kind="inline_dict",
                values=(
                    (
                        "robot_description",
                        uncarried(
                            f"{executable} {shlex.quote(model)} use_sim:=true",
                            "command_substitution",
                        ),
                    ),
                ),
            ),
        ),
        argv=("-d", rviz, "--params-file", params, other, other),
        site=site,
    )
    reads = _file_reads(node)
    assert {(read.path, read.role) for read in reads} == {
        (model, "robot_model"),
        (rviz, "rviz_configuration"),
        (params, "parameters_file"),
        (other, "node_argument"),
    }
    assert len(reads) == 4
    assert all(read.site == site for read in reads)


@pytest.mark.ros
def test_command_arguments_keep_their_slots_and_existing_empty_arguments(tmp_path):
    entry = tmp_path / "arguments.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.substitutions import Command\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([Node(package='rviz2', executable='rviz2', "
        "arguments=['', '-d', Command(['printf view']), '/share/other.txt'])])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert not walk.exceptions
    assert walk.nodes[0].argv == ("", "-d", "", "/share/other.txt")
    assert [(read.path, read.role) for read in walk.nodes[0].file_reads] == [
        ("/share/other.txt", "node_argument")
    ]


@pytest.mark.ros
def test_a_command_include_is_a_blind_spot_and_the_walk_continues(tmp_path):
    entry = tmp_path / "include.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import IncludeLaunchDescription\n"
        "from launch.substitutions import Command\n"
        "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([IncludeLaunchDescription("
        "PythonLaunchDescriptionSource(Command(['printf child.launch.py']))), "
        "Node(package='demo_nodes_cpp', executable='talker')])\n"
    )
    walk = launch_exec.walk(str(entry))
    assert not walk.exceptions
    assert len(walk.nodes) == 1
    assert not walk.includes[0].resolved
    assert walk.includes[0].reason == "command_substitution"


# stem, entry point relative to ws_launch, arguments, environment
CASES = [
    ("composition", "src/lx_bringup/launch/composition.launch.py", {}, {}),
    ("conditions.default", "src/lx_bringup/launch/conditions.launch.py", {}, {}),
    (
        "conditions.enabled",
        "src/lx_bringup/launch/conditions.launch.py",
        {"enable_extra": "true"},
        {},
    ),
    ("default_name", "src/lx_bringup/launch/default_name.launch.py", {}, {}),
    ("env_required.missing", "src/lx_bringup/launch/env_required.launch.py", {}, {}),
    (
        "env_required.set",
        "src/lx_bringup/launch/env_required.launch.py",
        {},
        {"LX_ROOT": "<ws_launch>"},
    ),
    ("event_gate", "src/lx_bringup/launch/event_gate.launch.py", {}, {}),
    ("exec_process", "src/lx_bringup/launch/exec_process.launch.py", {}, {}),
    ("group_scope", "src/lx_bringup/launch/group_scope.launch.py", {}, {}),
    ("include_child", "src/lx_bringup/launch/include_child.launch.py", {}, {}),
    ("include_parent", "src/lx_bringup/launch/include_parent.launch.py", {}, {}),
    ("nav_stack", "src/lx_bringup/launch/nav_stack.launch.py", {}, {}),
    ("no_default_arg.missing", "src/lx_bringup/launch/no_default_arg.launch.py", {}, {}),
    (
        "no_default_arg.supplied",
        "src/lx_bringup/launch/no_default_arg.launch.py",
        {"params_file": "src/lx_bringup/config/order_base.yaml"},
        {},
    ),
    ("opaque_probe.no_map", "src/lx_bringup/launch/opaque_probe.launch.py", {}, {}),
    (
        "opaque_probe.with_map",
        "src/lx_bringup/launch/opaque_probe.launch.py",
        {"map_base": "<tmp>/yard"},
        {},
    ),
    ("param_order", "src/lx_bringup/launch/param_order.launch.py", {}, {}),
    ("plain_node", "src/lx_bringup/launch/plain_node.launch.py", {}, {}),
    ("rewritten", "src/lx_bringup/launch/rewritten.launch.py", {}, {}),
    ("sensors_only", "src/lx_sensors/launch/sensors_only.launch.py", {}, {}),
    ("timer_chain", "src/lx_bringup/launch/timer_chain.launch.py", {}, {}),
]


# ---------------------------------------------------------------------------
# helpers


@pytest.fixture(scope="session")
def ws_launch(fixtures_dir: Path) -> Path:
    path = fixtures_dir / WS
    if not path.is_dir():
        pytest.skip("the launch fixture workspace is not present")
    return path


@pytest.fixture(scope="session")
def expected_dir(fixtures_dir: Path) -> Path:
    return fixtures_dir / "expected" / "launch"


def _shorten(value: str, base: Path, tmp: Path | None) -> str:
    text = str(value)
    if text == str(base):
        return "<ws_launch>"
    for prefix, label in ((str(base) + "/", ""), (str(tmp) + "/" if tmp else None, "<tmp>/")):
        if prefix and text.startswith(prefix):
            return label + text[len(prefix) :]
    if tmp is not None and text == str(tmp):
        return "<tmp>"
    return text


def _expand(value: str, base: Path, tmp: Path) -> str:
    text = str(value)
    if text == "<ws_launch>":
        return str(base)
    if text.startswith("<tmp>/"):
        return str(tmp / text[len("<tmp>/") :])
    if text.startswith("src/"):
        return str(base / text)
    return text


def _fqn(namespace: str, name: str | None) -> str | None:
    if name is None or not namespace:
        return None
    if namespace == "/":
        return "/" + name
    return f"{namespace.rstrip('/')}/{name}"


def _project(walk: records.LaunchWalk, base: Path, tmp: Path) -> dict:
    short = lambda value: _shorten(value, base, tmp)  # noqa: E731
    nodes = []
    for node in walk.nodes:
        if node.attrs.get("unreachable"):
            # Only a condition that really evaluated false may take a node out of the
            # comparison. Without this the projection would hide a node the resolver dropped
            # because it could not evaluate the condition at all.
            assert [gate.outcome for gate in node.gates if gate.outcome == "false"], node
            continue
        nodes.append(
            {
                "argv": [short(item) for item in node.argv],
                "container": node.container,
                "executable": node.executable,
                "form": node.form,
                "fqn": _fqn(node.namespace, node.name),
                "gates": [
                    {"mechanism": gate.mechanism, "period": gate.period, "target": gate.target}
                    for gate in node.gates
                ],
                "name": node.name,
                "name_source": node.name_source,
                "namespace": node.namespace,
                "params": [
                    {
                        "kind": item.kind,
                        "order": item.order,
                        "path": short(item.path) if item.path else None,
                        "rewrites": [list(pair) for pair in item.rewrites],
                        "root_key": item.root_key,
                        "source_path": short(item.source_path) if item.source_path else None,
                        "values": [list(pair) for pair in item.values],
                    }
                    for item in node.params
                ],
                "pkg": node.pkg,
                "plugin_class": node.plugin_class,
                "remaps": [
                    {"from": remap.frm, "scope": remap.scope, "to": remap.to}
                    for remap in node.remaps
                ],
            }
        )
    declarations = []
    for item in walk.arg_declarations:
        row = {"name": item.name, "default": short(item.default) if item.default else item.default}
        if item.description:
            row["description"] = item.description
        declarations.append(row)
    return {
        "arg_declarations": declarations,
        "args": [[key, short(value)] for key, value in walk.args],
        "entry": short(walk.entry_path),
        "env": [[key, short(value)] for key, value in walk.env],
        "exceptions": [
            {
                "exc_type": item.exc_type,
                "message": item.message,
                "path": short(item.path),
                "phase": item.phase,
            }
            for item in walk.exceptions
        ],
        "loadable": walk.loadable,
        "nodes": nodes,
        "strategy": walk.strategy,
    }


def _wanted(payload: dict) -> dict:
    """The expected file with the fields the README calls prose or advisory removed."""
    dropped = ("case", "note", "predicates")
    out = {key: value for key, value in payload.items() if key not in dropped}
    for node in out.get("nodes", ()):
        node.pop("conditions", None)
        for item in node.get("params", ()):
            item.pop("note", None)
    return out


def _run(case, ws_launch: Path, tmp_path: Path) -> records.LaunchWalk:
    _stem, entry, args, env = case
    if entry.endswith("opaque_probe.launch.py") and "map_base" in args:
        (tmp_path / "yard.posegraph").write_text("map\n", encoding="utf-8")
    return launch_exec.walk(
        str(ws_launch / entry),
        args=tuple((key, _expand(value, ws_launch, tmp_path)) for key, value in args.items()),
        env=tuple((key, _expand(value, ws_launch, tmp_path)) for key, value in env.items()),
        ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        timeout=90.0,
        launch_sandbox="off",
    )


# ---------------------------------------------------------------------------
# the fixture sweep


@pytest.mark.ros
@pytest.mark.parametrize("case", CASES, ids=[case[0] for case in CASES])
def test_walk_matches_expected(case, ws_launch: Path, expected_dir: Path, tmp_path: Path) -> None:
    stem = case[0]
    expected = _wanted(json.loads((expected_dir / f"{stem}.json").read_text(encoding="utf-8")))
    walk = _run(case, ws_launch, tmp_path)
    assert _project(walk, ws_launch, tmp_path) == expected


@pytest.mark.ros
def test_every_expected_file_has_a_case(expected_dir: Path) -> None:
    """No expected answer may sit unchecked, and the refusal is checked on its own."""
    on_disk = {path.stem for path in expected_dir.glob("*.json")}
    covered = {case[0] for case in CASES} | {"legacy_ros1"}
    assert on_disk == covered


def test_ros1_xml_is_refused_by_shape(ws_launch: Path, expected_dir: Path) -> None:
    """Refused without a child, so the walk never needs ROS to say no."""
    expected = _wanted(json.loads((expected_dir / "legacy_ros1.json").read_text(encoding="utf-8")))
    walk = launch_exec.walk(str(ws_launch / "src/lx_bringup/launch/legacy_ros1.launch"))
    assert _project(walk, ws_launch, ws_launch) == expected
    assert walk.notes


def test_missing_entry_point_is_an_import_exception(ws_launch: Path) -> None:
    walk = launch_exec.walk(str(ws_launch / "src/lx_bringup/launch/absent_profile.launch.py"))
    assert walk.strategy == "refused"
    assert walk.loadable is False
    assert [item.phase for item in walk.exceptions] == ["import"]


# ---------------------------------------------------------------------------
# properties the expected files do not carry


@pytest.mark.ros
def test_probe_outcome_is_recorded(ws_launch: Path, tmp_path: Path) -> None:
    """The filesystem probe that picks the executable is visible in the run record."""
    with_map = _run(
        (
            "opaque_probe.with_map",
            "src/lx_bringup/launch/opaque_probe.launch.py",
            {"map_base": "<tmp>/yard"},
            {},
        ),
        ws_launch,
        tmp_path,
    )
    posegraph = [key for key, value in with_map.predicates.items() if key.endswith(".posegraph")]
    assert posegraph, with_map.predicates
    assert all(with_map.predicates[key] for key in posegraph)
    no_map = _run(
        ("opaque_probe.no_map", "src/lx_bringup/launch/opaque_probe.launch.py", {}, {}),
        ws_launch,
        tmp_path,
    )
    assert not any(no_map.predicates.get(key) for key in no_map.predicates)


@pytest.mark.ros
def test_false_condition_node_is_kept(ws_launch: Path, tmp_path: Path) -> None:
    """The node the argument switched off still exists, with the gate that killed it."""
    walk = _run(CASES[1], ws_launch, tmp_path)
    unreachable = [node for node in walk.nodes if node.attrs.get("unreachable")]
    assert [node.name for node in unreachable] == ["if_node"]
    assert unreachable[0].gates[-1].mechanism == "IfCondition"
    assert unreachable[0].gates[-1].outcome == "false"
    assert "enable_extra" in unreachable[0].conditions[0]


@pytest.mark.ros
@pytest.mark.parametrize("case", CASES, ids=[case[0] for case in CASES])
def test_role_content_carries_no_machine_path(case, ws_launch: Path, tmp_path: Path) -> None:
    """Identity content decides a node id, so nothing in it may name one machine or one run."""
    walk = _run(case, ws_launch, tmp_path)
    for node in walk.nodes:
        for item in node.role_content:
            value = item.split(":", 1)[-1]
            assert str(ws_launch) not in value, (node.executable, item)
            assert not value.startswith(str(Path(tmp_path).parent)), (node.executable, item)


@pytest.mark.ros
def test_a_condition_the_walk_cannot_evaluate_is_not_a_false_one(tmp_path: Path) -> None:
    """An unknown outcome must not read as a node that never starts."""
    entry = tmp_path / "unknown_condition.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.conditions import IfCondition\n"
        "from launch.substitutions import LaunchConfiguration\n"
        "from launch_ros.actions import Node\n"
        "\n"
        "\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        Node(package='lx_sensors', executable='scan_relay', name='unknowable',\n"
        "             condition=IfCondition(LaunchConfiguration('never_declared'))),\n"
        "        Node(package='lx_sensors', executable='imu_driver',\n"
        "             name=LaunchConfiguration('also_never_declared')),\n"
        "    ])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry), ros_setup=[path for path in ROS_SETUP if Path(path).is_file()], timeout=90.0
    )
    gated = next(node for node in walk.nodes if node.executable == "scan_relay")
    assert gated.gates[-1].outcome == "unknown"
    assert "unreachable" not in gated.attrs
    unperformed = next(node for node in walk.nodes if node.executable == "imu_driver")
    assert unperformed.name is None
    assert unperformed.name_source == "unknown"


@pytest.mark.ros
def test_an_unstated_namespace_is_the_root(ws_launch: Path, tmp_path: Path) -> None:
    """launch_ros states no namespace by writing a sentinel, and it means the root."""
    case = ("default_name", "src/lx_bringup/launch/default_name.launch.py", {}, {})
    node = _run(case, ws_launch, tmp_path).nodes[0]
    assert node.namespace == "/"
    assert node.name is None
    assert node.name_source == "executable_default"
    assert node.attrs["namespace_sentinel"] == substitutions.UNSPECIFIED_NAMESPACE
    assert node.attrs["name_sentinel"] == substitutions.UNSPECIFIED_NAME


@pytest.mark.ros
@pytest.mark.parametrize("case", CASES, ids=[case[0] for case in CASES])
def test_the_sentinel_text_never_leaves_the_child(case, ws_launch: Path, tmp_path: Path) -> None:
    """The raw sentinel is auditable in attrs and nowhere else, since it names nothing."""
    walk = _run(case, ws_launch, tmp_path)
    payload = launch_exec.encode_walk(walk)
    for node in payload["nodes"]:
        audit = node.pop("attrs")
        text = json.dumps(node)
        for sentinel in (substitutions.UNSPECIFIED_NAME, substitutions.UNSPECIFIED_NAMESPACE):
            assert sentinel not in text, node["executable"]
        for key, value in audit.items():
            if value in (substitutions.UNSPECIFIED_NAME, substitutions.UNSPECIFIED_NAMESPACE):
                assert key in ("name_sentinel", "namespace_sentinel")


@pytest.mark.ros
def test_a_process_reads_its_parameters_through_the_argv_parser(
    ws_launch: Path, tmp_path: Path
) -> None:
    """One rule for argv precedence: the process node is the parser's own answer."""
    case = ("exec_process", "src/lx_bringup/launch/exec_process.launch.py", {}, {})
    node = _run(case, ws_launch, tmp_path).nodes[0]
    parsed = ros_args.parse_ros_args(node.argv)
    assert [item.kind for item in node.params] == [
        "ros_args_file" if kind == ros_args.INPUT_FILE else "ros_args_assign"
        for kind, _payload in parsed.inputs
    ]
    assert node.params[-1].kind == "ros_args_file", "the file is the last input, so it wins"
    assert node.name == "cropper"
    assert node.name_source == "argv"
    assert node.namespace == "/prep"


@pytest.mark.ros
def test_a_process_namespace_is_claimed_only_from_argv(tmp_path: Path) -> None:
    """launch pushes no namespace into a process, so root here would be a value nobody read."""
    entry = tmp_path / "process_namespace.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import ExecuteProcess, GroupAction, SetEnvironmentVariable\n"
        "from launch_ros.actions import PushRosNamespace\n"
        "\n"
        "\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        SetEnvironmentVariable('ROS_NAMESPACE', '/from_env'),\n"
        "        GroupAction([\n"
        "            PushRosNamespace('grouped'),\n"
        "            ExecuteProcess(cmd=['ros2', 'run', 'lx_sensors', 'scan_relay',\n"
        "                                '--ros-args', '-p', 'rate:=20']),\n"
        "            ExecuteProcess(cmd=['ros2', 'run', 'lx_sensors', 'imu_driver',\n"
        "                                '--ros-args', '-r', '__ns:=/stated']),\n"
        "        ]),\n"
        "    ])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry), ros_setup=[path for path in ROS_SETUP if Path(path).is_file()], timeout=90.0
    )
    found = {node.executable: node.namespace for node in walk.nodes}
    assert found == {"scan_relay": "", "imu_driver": "/stated"}


@pytest.mark.ros
@pytest.mark.parametrize("description_condition", [None, "true", "false"])
@pytest.mark.parametrize("map_value", [None, "map.yaml"])
def test_load_composable_conditions_are_kept(
    tmp_path: Path, description_condition: str | None, map_value: str | None
) -> None:
    entry = tmp_path / "conditional_components.launch.py"
    own = f", condition=IfCondition('{description_condition}')" if description_condition else ""
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.conditions import IfCondition\n"
        "from launch.substitutions import (\n"
        "    EqualsSubstitution, NotEqualsSubstitution, LaunchConfiguration)\n"
        "from launch_ros.actions import LoadComposableNodes\n"
        "from launch_ros.descriptions import ComposableNode\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        LoadComposableNodes(target_container='container',\n"
        "            condition=IfCondition(test(LaunchConfiguration('map'), '')),\n"
        "            composable_node_descriptions=[ComposableNode(\n"
        f"                package='demo', plugin='demo::Map', name='map_server'{own})])\n"
        "        for test in (EqualsSubstitution, NotEqualsSubstitution)\n"
        "    ])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry),
        args=[("map", map_value)] if map_value else [],
        ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        timeout=90.0,
    )
    assert len(walk.nodes) == 2, walk.exceptions
    for node, outcome, comparison in zip(
        walk.nodes,
        ["false", "true"] if map_value else ["unknown", "unknown"],
        ["EqualsSubstitution", "NotEqualsSubstitution"],
        strict=True,
    ):
        assert node.name == "map_server"
        assert node.gates[0].mechanism == "IfCondition"
        assert node.gates[0].outcome == outcome
        assert comparison in node.conditions[0]
        assert node.conditions[0] == node.gates[0].expression
        assert len(node.conditions) == 1 + bool(description_condition)
        if description_condition:
            assert node.gates[1].outcome == description_condition
            assert node.conditions[1] == node.gates[1].expression
        assert bool(node.attrs.get("unreachable")) == (
            outcome == "false" or description_condition == "false"
        )


@pytest.mark.ros
def test_a_container_namespace_is_read_only_after_it_expanded(tmp_path: Path) -> None:
    """The container sentinel before expansion means not computed yet, never the root."""
    entry = tmp_path / "late_container.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import TimerAction\n"
        "from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes\n"
        "from launch_ros.descriptions import ComposableNode\n"
        "\n"
        "CONTAINER = ComposableNodeContainer(\n"
        "    name='pipe_container', namespace='/sensors', package='rclcpp_components',\n"
        "    executable='component_container', composable_node_descriptions=[])\n"
        "\n"
        "\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        LoadComposableNodes(target_container=CONTAINER,\n"
        "                            composable_node_descriptions=[ComposableNode(\n"
        "                                package='lx_sensors', plugin='lx::Filter',\n"
        "                                name='filter')]),\n"
        "        TimerAction(period=1.0, actions=[CONTAINER]),\n"
        "    ])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry), ros_setup=[path for path in ROS_SETUP if Path(path).is_file()], timeout=90.0
    )
    component = next(node for node in walk.nodes if node.form == "composable_node")
    assert component.container != "/pipe_container", "the container sits under /sensors"


@pytest.mark.ros
def test_entity_path_carries_the_action_trail(ws_launch: Path, tmp_path: Path) -> None:
    case = ("timer_chain", "src/lx_bringup/launch/timer_chain.launch.py", {}, {})
    walk = _run(case, ws_launch, tmp_path)
    trails = {node.name: node.entity_path for node in walk.nodes}
    assert trails["late_node"] == ("TimerAction(5s)", "Node(late_node)")
    assert trails["deep_node"] == (
        "OnProcessExit(stage_one)",
        "OnProcessExit(stage_two)",
        "Node(deep_node)",
    )


@pytest.mark.ros
def test_include_is_recorded_with_its_arguments(ws_launch: Path, tmp_path: Path) -> None:
    walk = _run(
        ("include_parent", "src/lx_bringup/launch/include_parent.launch.py", {}, {}),
        ws_launch,
        tmp_path,
    )
    assert len(walk.includes) == 1
    include = walk.includes[0]
    assert Path(include.path).name == "include_child.launch.py"
    assert include.args == (("child_name", "relay_child"), ("child_rate", "30.0"))


@pytest.mark.ros
def test_workspace_classifies_the_roots(ws_launch: Path) -> None:
    """With a workspace in hand every site carries its root and every file its tree."""
    from rosio.config import RootSpec
    from rosio.workspace import Workspace

    workspace = Workspace.discover(
        [RootSpec("src", str(ws_launch / "src")), RootSpec("install", str(ws_launch / "install"))]
    )
    walk = launch_exec.walk(
        str(ws_launch / "src/lx_bringup/launch/plain_node.launch.py"),
        ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        workspace=workspace,
    )
    assert walk.entry_pkg == "lx_bringup"
    node = walk.nodes[0]
    assert node.site is not None and node.site.root == "src"
    assert node.params[0].kind == "repo_file"


@pytest.mark.ros
def test_two_walks_agree(ws_launch: Path, tmp_path: Path) -> None:
    """Same inputs, same bytes: nothing in a walk may depend on iteration order."""
    case = ("nav_stack", "src/lx_bringup/launch/nav_stack.launch.py", {}, {})
    first = launch_exec.encode_walk(_run(case, ws_launch, tmp_path))
    second = launch_exec.encode_walk(_run(case, ws_launch, tmp_path))
    assert json.dumps(first, sort_keys=True) == json.dumps(second, sort_keys=True)


@pytest.mark.ros
def test_round_trip_through_the_codec(ws_launch: Path, tmp_path: Path) -> None:
    case = ("event_gate", "src/lx_bringup/launch/event_gate.launch.py", {}, {})
    walk = _run(case, ws_launch, tmp_path)
    again = launch_exec.decode_walk(launch_exec.encode_walk(walk))
    assert launch_exec.encode_walk(again) == launch_exec.encode_walk(walk)


def test_the_child_request_goes_through_the_pinned_encoder(ws_launch: Path, monkeypatch) -> None:
    """model.dumps is the only encoder in the codebase, so the request the child is handed is
    sorted and stable rather than whatever json.dumps happens to emit."""
    from rosio import model

    seen: dict[str, str] = {}

    def fake_run(_self, _argv, _env, text, _timeout):
        seen["input"] = text
        raise OSError("no child in this test")

    monkeypatch.setattr(launch_exec.Children, "run", fake_run)
    launch_exec.walk(
        str(ws_launch / "src/lx_bringup/launch/plain_node.launch.py"), launch_sandbox="off"
    )
    assert seen["input"] == model.dumps(json.loads(seen["input"]))


@pytest.mark.ros
def test_available_reports_the_sourced_environment() -> None:
    assert launch_exec.available([path for path in ROS_SETUP if Path(path).is_file()]) is True


# ---------------------------------------------------------------------------
# the text only walk, and the gap it leaves


@pytest.mark.parametrize(
    ("entry", "found"),
    [
        ("src/lx_bringup/launch/plain_node.launch.py", 1),
        ("src/lx_bringup/launch/opaque_probe.launch.py", 0),
        ("src/lx_bringup/launch/event_gate.launch.py", 2),
        ("src/lx_bringup/launch/nav_stack.launch.py", 6),
    ],
)
def test_ast_walk_reports_only_what_it_sees(ws_launch: Path, entry: str, found: int) -> None:
    walk = launch_exec.walk_ast(str(ws_launch / entry))
    assert walk.strategy == "ast"
    assert len(walk.nodes) == found


def test_ast_walk_never_invents_a_namespace(ws_launch: Path) -> None:
    walk = launch_exec.walk_ast(str(ws_launch / "src/lx_bringup/launch/default_name.launch.py"))
    assert [(node.name, node.namespace, node.name_source) for node in walk.nodes] == [
        (None, "", "executable_default")
    ]


@pytest.mark.ros
def test_executed_walk_finds_nodes_the_ast_walk_cannot(ws_launch: Path, tmp_path: Path) -> None:
    """The whole thesis of the executed walk, on the two fixtures that hide their payload."""
    for entry in (
        "src/lx_bringup/launch/opaque_probe.launch.py",
        "src/lx_bringup/launch/event_gate.launch.py",
    ):
        executed = launch_exec.walk(
            str(ws_launch / entry),
            ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        )
        static = launch_exec.walk_ast(str(ws_launch / entry))
        assert len(executed.nodes) > 0
        assert len(executed.nodes) >= len(static.nodes)
    del tmp_path


CHAIN = "src/lx_bringup/launch/slam_chain.launch.py"


@pytest.mark.ros
@pytest.mark.slow
def test_the_full_chain_resolves_through_the_function_and_the_gate(
    ws_launch: Path, tmp_path: Path
) -> None:
    """slam_chain puts every node behind an OpaqueFunction and an OnProcessExit together.

    The executable is chosen by a filesystem probe inside the function, and the nodes are
    only in the on_exit list, so a walk that loses either half loses both nodes.
    """
    (tmp_path / "yard.posegraph").write_text("map\n", encoding="utf-8")
    walk = launch_exec.walk(
        str(ws_launch / CHAIN),
        args=(("map_base", str(tmp_path / "yard")),),
        ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        timeout=90.0,
        launch_sandbox="off",
    )
    assert walk.strategy == "executed"
    assert walk.exceptions == []
    names = sorted(node.name or node.executable for node in walk.nodes)
    assert "slam_toolbox" in names
    assert "grid_trimmer" in names
    assert len(walk.nodes) > len(launch_exec.walk_ast(str(ws_launch / CHAIN)).nodes)
    slam = next(node for node in walk.nodes if node.name == "slam_toolbox")
    assert slam.executable == "localization_slam", "the filesystem probe ran"
    assert [gate.mechanism for gate in slam.gates] == ["OnProcessExit"]
    assert ("/map", "/slam_map") in [(remap.frm, remap.to) for remap in slam.remaps]


@pytest.mark.ros
@pytest.mark.slow
def test_the_chain_probe_reads_the_absent_map_too(ws_launch: Path, tmp_path: Path) -> None:
    """Same file, no posegraph on disk, so the function picks the other executable."""
    walk = launch_exec.walk(
        str(ws_launch / CHAIN),
        args=(("map_base", str(tmp_path / "yard")),),
        ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        timeout=90.0,
    )
    slam = next(node for node in walk.nodes if node.name == "slam_toolbox")
    assert slam.executable == "async_slam"
    assert [gate.mechanism for gate in slam.gates] == ["OnProcessExit"]


# ---------------------------------------------------------------------------
# whether the launch machinery can be used at all


def _payload(**overrides) -> dict:
    payload = {
        "ok": False,
        "module": "lark",
        "exc_type": "ModuleNotFoundError",
        "message": "No module named 'lark'",
        "venv": True,
        "system_site": False,
        "ros": True,
    }
    payload.update(overrides)
    return payload


def test_a_virtual_environment_that_hides_system_packages_is_named_with_its_flag() -> None:
    """The common failure, and the one that produced a hollow atlas with nothing said."""
    report = launch_exec.import_report(_payload())
    assert report.ok is False
    assert report.module == "lark"
    assert "--system-site-packages" in report.fix
    assert report.note.startswith(launch_exec.NOTE_UNAVAILABLE)
    assert "No module named 'lark'" in report.note
    assert report.fix in report.summary


def test_an_unsourced_environment_is_told_to_source_a_setup_file_first() -> None:
    report = launch_exec.import_report(_payload(module="launch", ros=False))
    assert report.fix == launch_exec.SETUP_FIX
    assert "--system-site-packages" not in report.fix


def test_a_sourced_interpreter_that_sees_system_packages_is_told_to_install_the_module() -> None:
    report = launch_exec.import_report(_payload(module="yaml", venv=False, system_site=True))
    assert report.fix == "install yaml into the interpreter that runs the launch walk"


def test_a_probe_that_did_not_run_at_all_still_reports_a_reason_and_a_fix() -> None:
    report = launch_exec.import_report({}, fallback="bash: python3: command not found")
    assert report.ok is False
    assert report.exc_type == "ChildFailed"
    assert "command not found" in report.message
    assert report.fix


def test_a_successful_probe_carries_nothing_to_repair() -> None:
    report = launch_exec.import_report({"ok": True})
    assert report.ok is True
    assert (report.note, report.fix, report.summary) == ("", "", "")


def test_available_is_the_answer_the_probe_gave(monkeypatch) -> None:
    monkeypatch.setattr(launch_exec, "_probes", {})
    monkeypatch.setattr(launch_exec, "probe", lambda *_setup: launch_exec.LaunchImport(ok=False))
    assert launch_exec.available(()) is False


def test_the_probe_script_reports_no_path_from_the_box_it_ran_on(capsys) -> None:
    """The answer is written into an atlas, and every path on this box is machine
    dependent, so the child sends back decisions and never the paths behind them."""
    exec(compile(launch_exec.PROBE_SCRIPT, "probe", "exec"), {})
    printed = capsys.readouterr().out
    payload = json.loads(printed.rsplit(launch_exec.MARKER, 1)[1])
    assert set(payload) <= {"ok", "module", "exc_type", "message", "venv", "system_site", "ros"}
    assert not [value for value in payload.values() if isinstance(value, str) and "/" in value]


# ---------------------------------------------------------------------------
# refusals say what is true about the file


def _write(tmp_path: Path, name: str, text: str) -> str:
    target = tmp_path / name
    target.write_text(text, encoding="utf-8")
    return str(target)


@pytest.mark.ros
def test_a_ros2_xml_launch_file_is_walked_through_its_frontend(tmp_path: Path) -> None:
    """jazzy ships launch_xml, so an XML entry point is read the way `ros2 launch` reads it
    instead of being refused by shape."""
    path = _write(
        tmp_path,
        "sensors.launch.xml",
        '<?xml version="1.0"?>\n<launch>\n  <arg name="ns" default="front"/>\n'
        '  <node pkg="lx" exec="driver" name="driver" namespace="$(var ns)"/>\n</launch>\n',
    )
    walk = launch_exec.walk(path)
    assert walk.loadable is True
    assert walk.exceptions == []
    assert [(node.pkg, node.executable, node.namespace, node.name) for node in walk.nodes] == [
        ("lx", "driver", "/front", "driver")
    ]


def test_a_ros1_xml_launch_file_keeps_its_own_note(tmp_path: Path) -> None:
    path = _write(
        tmp_path,
        "legacy.launch",
        '<launch>\n  <node pkg="lx" type="driver" name="driver"/>\n</launch>\n',
    )
    walk = launch_exec.walk(path)
    assert walk.loadable is False
    assert walk.notes == [launch_exec.NOTE_ROS1]


def test_two_includes_that_did_not_fold_keep_two_expressions(tmp_path: Path) -> None:
    """One expression for both is one blind spot for a collapse that took two files."""
    path = _write(
        tmp_path,
        "parent.launch.py",
        "from launch import LaunchDescription\n"
        "from launch.actions import IncludeLaunchDescription\n"
        "\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        IncludeLaunchDescription(source(one_dir, 'slam.launch.py')),\n"
        "        IncludeLaunchDescription(source(two_dir, 'navigation.launch.py')),\n"
        "    ])\n",
    )
    walk = launch_exec.walk_ast(path)
    assert [item.resolved for item in walk.includes] == [False, False]
    paths = [item.path for item in walk.includes]
    assert len(set(paths)) == 2
    assert all("slam.launch.py" in text or "navigation.launch.py" in text for text in paths)
    assert not any(" object at 0x" in text for text in paths)


# ---------------------------------------------------------------------------
# what the child sends back about an include


def test_a_location_that_did_not_stringify_loses_its_address() -> None:
    from rosio.resolve import _launch_child

    def source(location):
        return type("Source", (), {"location": location})()

    assert _launch_child._location(source("/opt/x/child.launch.py")) == (
        "/opt/x/child.launch.py",
        True,
    )
    text, resolved = _launch_child._location(
        source("<launch.substitutions.text_substitution.TextSubstitution object at 0xffff9c>")
    )
    assert resolved is False
    assert text == "<launch.substitutions.text_substitution.TextSubstitution>"


@pytest.mark.ros
def test_an_include_the_condition_switched_off_is_not_a_blind_spot(tmp_path: Path) -> None:
    """launch itself decided not to include the file, so nothing was missed and the path
    was never expanded. A blind spot here claims a hole the profile does not have."""
    entry = tmp_path / "gated.launch.py"
    entry.write_text(
        "import os\n"
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription\n"
        "from launch.conditions import IfCondition\n"
        "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
        "from launch.substitutions import LaunchConfiguration\n"
        "\n"
        "def generate_launch_description():\n"
        "    here = os.path.dirname(os.path.realpath(__file__))\n"
        "    return LaunchDescription([\n"
        "        DeclareLaunchArgument('slam', default_value='False'),\n"
        "        IncludeLaunchDescription(\n"
        "            PythonLaunchDescriptionSource(os.path.join(here, 'child.launch.py')),\n"
        "            condition=IfCondition(LaunchConfiguration('slam')),\n"
        "        ),\n"
        "    ])\n",
        encoding="utf-8",
    )
    (tmp_path / "child.launch.py").write_text(
        "from launch import LaunchDescription\n\n"
        "def generate_launch_description():\n    return LaunchDescription([])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry),
        ros_setup=[path for path in ROS_SETUP if Path(path).is_file()],
        timeout=90.0,
    )
    assert walk.exceptions == []
    assert walk.includes == []
    assert walk.notes == ["an include is switched off by IfCondition(LaunchConfiguration('slam'))"]


@pytest.mark.ros
def test_a_component_takes_the_group_parameters_and_remaps_first(tmp_path: Path) -> None:
    """launch_ros loads a component with the group scoped `SetParameter` values and `SetRemap`
    rules before its own. A composed nav2 bringup sets `use_sim_time` for the whole group this
    way, and every component lost it."""
    entry = tmp_path / "group_components.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import GroupAction\n"
        "from launch_ros.actions import LoadComposableNodes, SetParameter, SetRemap\n"
        "from launch_ros.descriptions import ComposableNode\n"
        "\n"
        "\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([\n"
        "        GroupAction([\n"
        "            SetParameter('use_sim_time', True),\n"
        "            SetRemap('cmd_vel', 'cmd_vel_nav'),\n"
        "            LoadComposableNodes(target_container='/nav_container', "
        "composable_node_descriptions=[\n"
        "                ComposableNode(package='lx_bringup', plugin='lx_bringup::Controller',\n"
        "                               name='controller', "
        "parameters=[{'controller_frequency': 20.0}],\n"
        "                               remappings=[('odom', '/odometry')]),\n"
        "            ]),\n"
        "        ]),\n"
        "    ])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry), ros_setup=[path for path in ROS_SETUP if Path(path).is_file()], timeout=90.0
    )
    component = next(node for node in walk.nodes if node.name == "controller")
    assert [dict(source.values) for source in component.params] == [
        {"use_sim_time": True},
        {"controller_frequency": 20.0},
    ]
    assert component.params[0].note == "SetParameter, group scoped"
    assert [(remap.frm, remap.to, remap.scope) for remap in component.remaps] == [
        ("cmd_vel", "cmd_vel_nav", "global"),
        ("odom", "/odometry", "node"),
    ]


def test_the_temp_directory_a_machine_uses_is_not_a_launch_predicate(monkeypatch) -> None:
    """A launch helper that creates a temp file reads `TMPDIR`. Recording it put a machine path
    into the run digest, so one tree extracted on two machines gave two atlases."""
    from rosio.resolve import _launch_child as launch_child

    monkeypatch.setenv("TMPDIR", "/somewhere/only/this/machine/has")
    monkeypatch.setenv("LX_ENABLE_EXTRA", "1")
    probes = launch_child._Probes()
    probes.install()
    try:
        import os

        os.environ.get("TMPDIR")
        os.environ.get("LX_ENABLE_EXTRA")
    finally:
        probes.restore()
    assert list(probes.seen) == ["env:LX_ENABLE_EXTRA=1"]


def test_the_sourced_environment_is_read_once_without_shell_state(tmp_path, monkeypatch):
    """Every child of one run gets the environment one sourcing produced, and a setup file
    that fails sends the children back to sourcing for themselves."""
    if shutil.which("bash") is None:
        pytest.skip("bash is not installed")
    setup = tmp_path / "setup.bash"
    setup.write_text("export ROSIO_SOURCED_ONCE=yes\n")
    monkeypatch.setattr(launch_exec, "_environments", {})
    env = launch_exec.sourced_environment([str(setup)])
    assert env is not None and env["ROSIO_SOURCED_ONCE"] == "yes"
    assert not launch_exec.SHELL_STATE & set(env)
    setup.write_text("export ROSIO_SOURCED_ONCE=again\n")
    assert launch_exec.sourced_environment([str(setup)]) is env
    command, child_env = launch_exec._python([str(setup)], "-c pass")
    assert command[:2] == ["bash", "-c"] and child_env is env

    broken = tmp_path / "broken.bash"
    broken.write_text("return 3\n")
    assert launch_exec.sourced_environment([str(broken)]) is None
    command, child_env = launch_exec._python([str(broken)], "-c pass")
    assert command[:2] == ["bash", "-lc"] and child_env is None


@pytest.mark.ros
@pytest.mark.parametrize(
    "start",
    [
        "subprocess.Popen(['touch', MARK])",
        "os.posix_spawnp('touch', ['touch', MARK], dict(os.environ))",
        "os.spawnlp(os.P_WAIT, 'touch', 'touch', MARK)",
        "os.execvp('touch', ['touch', MARK])",
    ],
)
def test_launch_code_cannot_start_a_process(start: str, tmp_path: Path) -> None:
    """The walk resolves a description, so every way of starting a process is refused."""
    mark = tmp_path / "started"
    entry = tmp_path / "spawn.launch.py"
    entry.write_text(
        "import os, subprocess\n"
        "from launch import LaunchDescription\n"
        f"MARK = {str(mark)!r}\n"
        "def generate_launch_description():\n"
        "    try:\n"
        f"        {start}\n"
        "    except RuntimeError:\n"
        "        pass\n"
        "    return LaunchDescription([])\n",
        encoding="utf-8",
    )
    walk = launch_exec.walk(
        str(entry), ros_setup=[path for path in ROS_SETUP if Path(path).is_file()], timeout=60.0
    )
    assert not mark.exists()
    assert any("a process start was refused" in note for note in walk.notes)


@pytest.mark.ros
def test_construction_lines_survive_the_sandbox(tmp_path: Path) -> None:
    child = tmp_path / "child.launch.py"
    child.write_text(
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([Node(package='demo_nodes_cpp', "
        "executable='talker', name='child')])\n"
    )
    entry = tmp_path / "parent.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import IncludeLaunchDescription\n"
        "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
        "from launch_ros.actions import Node, ComposableNodeContainer\n"
        "from launch_ros.descriptions import ComposableNode\n"
        "def generate_launch_description():\n"
        "    first = Node(package='demo_nodes_cpp', executable='talker', name='first')\n"
        "    second = Node(package='demo_nodes_cpp', executable='listener', name='second')\n"
        "    include = IncludeLaunchDescription("
        "PythonLaunchDescriptionSource(__file__.replace('parent', 'child')))\n"
        "    component = ComposableNode(package='composition', "
        "plugin='composition::Talker', name='component')\n"
        "    container = ComposableNodeContainer(package='rclcpp_components', "
        "executable='component_container', name='container', namespace='', "
        "composable_node_descriptions=[component])\n"
        "    return LaunchDescription([first, second, include, container])\n"
    )
    result = launch_exec.walk(str(entry))
    assert not result.exceptions
    assert {node.name: node.site.line_hint for node in result.nodes} == {
        "first": 7,
        "second": 8,
        "child": 4,
        "component": 10,
        "container": 11,
    }
    assert next(node for node in result.nodes if node.name == "child").site.path == str(child)
    assert result.includes[0].site.line_hint == 9
    assert result.includes[0].entity_path == ("Include(child.launch.py)",)
    assert launch_exec.encode_walk(result) == launch_exec.encode_walk(launch_exec.walk(str(entry)))


@pytest.mark.ros
@pytest.mark.parametrize("phase", ["import", "describe", "visit", "escape"])
def test_construction_hooks_are_restored_after_a_launch_raises(tmp_path, monkeypatch, phase):
    from launch.action import Action
    from launch.substitutions import Command
    from launch_ros.descriptions import ComposableNode
    from launch_ros.parameter_descriptions import ParameterValue

    from rosio.resolve._launch_child import Walker

    originals = (Action.__init__, ComposableNode.__init__, Command.perform, ParameterValue.evaluate)
    entry = tmp_path / "raises.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import OpaqueFunction\n"
        "def fail(*args, **kwargs):\n"
        "    raise RuntimeError('launch failed')\n"
        + ("fail()\n" if phase == "import" else "")
        + "def generate_launch_description():\n"
        + (
            "    fail()\n"
            if phase == "describe"
            else "    return LaunchDescription([OpaqueFunction(function=fail)])\n"
        )
    )
    walker = Walker({"entry_path": str(entry)})
    if phase == "escape":

        def fail(description):
            raise RuntimeError("walk failed")

        monkeypatch.setattr(walker, "_walk", fail)
        with pytest.raises(RuntimeError, match="walk failed"):
            walker.run()
    else:
        result = walker.run()
        assert any(row.phase == phase for row in result.exceptions)
    assert (
        Action.__init__,
        ComposableNode.__init__,
        Command.perform,
        ParameterValue.evaluate,
    ) == originals


@pytest.mark.ros
def test_construction_line_uses_the_innermost_frame_in_the_launch_file(tmp_path):
    entry = tmp_path / "helper.launch.py"
    entry.write_text(
        "from launch import LaunchDescription\n"
        "from launch_ros.actions import Node\n"
        "def make_node():\n"
        "    return Node(package='demo_nodes_cpp', executable='talker', name='helper')\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([make_node()])\n"
    )
    result = launch_exec.walk(str(entry))
    assert not result.exceptions
    assert result.nodes[0].site.line_hint == 4


def test_a_stopped_run_kills_its_child_and_starts_no_other(tmp_path: Path, monkeypatch) -> None:
    import sys
    import threading
    import time

    from rosio.errors import CollectorError

    entry = tmp_path / "slow.launch.py"
    entry.write_text("value = 1\n", encoding="utf-8")
    sleeper = [sys.executable, "-c", "import time; time.sleep(60)"]
    monkeypatch.setattr(launch_exec, "_python", lambda _setup, _script: (sleeper, None))
    children = launch_exec.Children()
    threading.Timer(0.5, children.stop).start()
    began = time.monotonic()
    walk = launch_exec.walk(str(entry), launch_sandbox="off", timeout=60, children=children)
    assert time.monotonic() - began < 10
    assert walk.strategy == "refused"
    with pytest.raises(CollectorError, match="stopped"):
        launch_exec.walk(str(entry), launch_sandbox="off", children=children)


def test_a_file_launch_code_opened_is_read_once_per_walk(tmp_path: Path) -> None:
    from rosio.resolve import _launch_child

    model_file = tmp_path / "robot.urdf"
    model_file.write_text("<robot/>", encoding="utf-8")
    models = _launch_child._ModelTexts()
    reads: list[str] = []
    original = models._open

    def counted(path, **kwargs):
        reads.append(path)
        return original(path, **kwargs)

    models._open = counted
    models.opened.add((str(model_file), None, None, None))
    for _node in range(3):
        assert models.texts() == {"<robot/>": {str(model_file)}}
    assert reads == [str(model_file)]


def test_only_a_package_no_install_root_holds_is_read_from_source(tmp_path: Path) -> None:
    from rosio.config import RootSpec
    from rosio.workspace import Workspace

    manifest = "<package format='3'><name>{}</name><version>0.0.1</version></package>"
    for directory, name in (
        ("src/built", "built"),
        ("src/unbuilt", "unbuilt"),
        ("install/share/built", "built"),
    ):
        (tmp_path / directory).mkdir(parents=True)
        (tmp_path / directory / "package.xml").write_text(manifest.format(name))
    workspace = Workspace.discover(
        [RootSpec("src", str(tmp_path / "src")), RootSpec("install", str(tmp_path / "install"))]
    )
    assert launch_exec.source_shares(workspace) == {"unbuilt": str(tmp_path / "src/unbuilt")}
    assert launch_exec.source_shares(None) == {}


ENVIRONMENT_READS = """
import os
from os import environ
from launch.substitutions import EnvironmentVariable

MODEL = os.environ['ROBOT_MODEL']


def generate_launch_description():
    lidar = environ["LIDAR_MODEL"]
    distro = os.environ.get('ROS_DISTRO')
    quiet = os.environ.get('QUIET', 'false')
    home = os.getenv('ROBOT_HOME')
    port = os.getenv('PORT', '9090')
    arm = EnvironmentVariable('ARM_MODEL')
    gripper = EnvironmentVariable(name='GRIPPER', default_value='none')
    again = os.environ['ROBOT_MODEL']
    if os.environ.get('X_FLAG'):
        os.environ['WRITTEN_HERE'] = '1'
    return [lidar, distro, quiet, home, port, arm, gripper, again]
"""


def test_environment_reads_name_every_variable_read_without_a_default():
    """`os.environ.get` and `os.getenv` answer None for an unset variable, so a file that
    reads one that way does not need it, and a variable the file sets is not read."""
    assert launch_exec.environment_reads(ENVIRONMENT_READS) == (
        "ROBOT_MODEL",
        "LIDAR_MODEL",
        "ARM_MODEL",
    )


def test_environment_reads_of_a_file_that_does_not_parse_are_empty():
    assert launch_exec.environment_reads("def broken(:") == ()
