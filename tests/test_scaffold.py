"""`rosio init`: the fallback configuration, written out and read back."""

from __future__ import annotations

import dataclasses
import os
import shutil
from pathlib import Path

import pytest
import yaml

from rosio import cli, scaffold
from rosio.config import CONFIG_NAME, RootSpec, default_config, load_config
from rosio.errors import UsageError

WS = Path(__file__).parent / "fixtures" / "ws_launch"


@pytest.fixture
def workspace(tmp_path: Path) -> Path:
    shutil.copytree(WS / "src", tmp_path / "src")
    return tmp_path


def test_the_written_file_reads_back_as_the_same_configuration(workspace: Path) -> None:
    target, written = scaffold.write(workspace)
    assert target == workspace / CONFIG_NAME
    read = load_config(str(target), workspace=str(workspace))
    assert [profile.slug for profile in read.profiles] == [p.slug for p in written.profiles]
    offered = scaffold.offered(default_config(str(workspace))).profiles
    inside = scaffold.started_inside(offered)
    assert [profile.slug for profile in read.profiles] == [
        p.slug for p in offered if p.slug not in inside
    ]
    assert {Path(spec.path).resolve() for spec in read.roots} == {
        Path(spec.path).resolve() for spec in written.roots
    }


def test_paths_inside_the_workspace_are_written_relative(workspace: Path) -> None:
    cfg = default_config(str(workspace))
    extra = RootSpec(kind="install", path=str(workspace / "install"))
    text = scaffold.render(dataclasses.replace(cfg, roots=(*cfg.roots, extra)), workspace)
    payload = yaml.safe_load(text)
    assert all(not Path(root["path"]).is_absolute() for root in payload["roots"])
    assert str(workspace) not in text
    assert text.startswith("# rosio configuration, written by `rosio init`.")


def test_the_default_roots_are_left_out_so_a_later_build_is_read(workspace: Path) -> None:
    """init wrote `roots: [src]`, and after colcon build the install tree stayed unread."""
    _target, _written = scaffold.write(workspace)
    text = (workspace / CONFIG_NAME).read_text(encoding="utf-8")
    payload = yaml.safe_load(text)
    assert "roots" not in payload and "ros_setup" not in payload
    assert "#   - kind: src" in text and "#     path: src" in text
    _installed(workspace / "install", "bot_bringup")
    read = load_config(str(workspace / CONFIG_NAME), workspace=str(workspace))
    assert [spec.kind for spec in read.roots] == ["src", "install"]


def test_an_included_launch_file_that_needs_arguments_is_not_offered(workspace: Path) -> None:
    launch = next((workspace / "src").rglob("*.launch.py"))
    base = launch.with_name("needs_args.launch.py")
    base.write_text(
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([DeclareLaunchArgument('description_file')])\n",
        encoding="utf-8",
    )
    alone = launch.with_name("alone.launch.py")
    alone.write_text(base.read_text(encoding="utf-8"), encoding="utf-8")
    launch.with_name("top.launch.py").write_text(_including("needs_args.launch.py"))
    text = scaffold.render(default_config(str(workspace)), workspace)
    assert "needs_args.launch.py" not in text
    assert "alone.launch.py declares these with no default" in text
    assert "  #     description_file: ''" in text


def _including(name: str, through_variable: bool = False, package: str = "lx_bringup") -> str:
    source = f'os.path.join(get_package_share_directory("{package}"), "launch", "{name}")'
    return (
        "import os\n"
        "from ament_index_python.packages import get_package_share_directory\n"
        "from launch import LaunchDescription\n"
        "from launch.actions import IncludeLaunchDescription\n"
        "from launch.launch_description_sources import PythonLaunchDescriptionSource\n"
        "def generate_launch_description():\n"
        + (f"    path = {source}\n" if through_variable else "")
        + "    return LaunchDescription([IncludeLaunchDescription(\n"
        + f"        PythonLaunchDescriptionSource({'path' if through_variable else source}))])\n"
    )


def test_only_a_real_include_hides_a_launch_file_that_needs_arguments(workspace: Path) -> None:
    """`sim.launch.py` counted as included wherever `nav_sim.launch.py` was, and a comment
    naming a file counted as including it."""
    launch = next((workspace / "src").rglob("*.launch.py"))
    needs = (
        "from launch import LaunchDescription\n"
        "from launch.actions import DeclareLaunchArgument\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([DeclareLaunchArgument('world')])\n"
    )
    for name in ("sim.launch.py", "nav_sim.launch.py", "noted.launch.py", "held.launch.py"):
        launch.with_name(name).write_text(needs)
    launch.with_name("top.launch.py").write_text(
        _including("nav_sim.launch.py") + "# noted.launch.py starts the same nodes\n"
    )
    launch.with_name("other.launch.py").write_text(
        _including("held.launch.py", through_variable=True)
    )
    text = scaffold.render(default_config(str(workspace)), workspace)
    assert "sim.launch.py declares these with no default" in text
    assert "noted.launch.py declares these with no default" in text
    assert "nav_sim.launch.py" not in text
    assert "held.launch.py" not in text


def test_an_existing_file_is_kept_unless_asked(workspace: Path) -> None:
    (workspace / CONFIG_NAME).write_text("roots: []\n", encoding="utf-8")
    with pytest.raises(UsageError, match="already exists"):
        scaffold.write(workspace)
    scaffold.write(workspace, force=True)
    assert "profiles:" in (workspace / CONFIG_NAME).read_text(encoding="utf-8")


def test_print_writes_nothing(workspace: Path, capsys: pytest.CaptureFixture[str]) -> None:
    assert cli.main(["init", str(workspace), "--print"]) == 0
    assert "profiles:" in capsys.readouterr().out
    assert not (workspace / CONFIG_NAME).exists()


def _installed(prefix: Path, package: str, launch: str = "bringup.launch.py") -> None:
    """One package as colcon and the distro packages install it: index marker and share."""
    index = prefix / "share" / "ament_index" / "resource_index" / "packages"
    index.mkdir(parents=True, exist_ok=True)
    (index / package).write_text("", encoding="utf-8")
    share = prefix / "share" / package
    (share / "launch").mkdir(parents=True)
    (share / "package.xml").write_text(
        f'<package format="3"><name>{package}</name></package>', encoding="utf-8"
    )
    (share / "launch" / launch).write_text("", encoding="utf-8")


def _profiles(cfg) -> dict[str, str]:
    return {
        profile.slug: f"{entry.pkg} {entry.target}"
        for profile in cfg.profiles
        for entry in profile.entrypoints
    }


@pytest.fixture
def merged(tmp_path: Path) -> Path:
    prefix = tmp_path / "jazzy"
    _installed(prefix, "alpha")
    _installed(prefix, "beta", "beta_launch.py")
    # /opt/ros/<distro> holds a src and launch files outside every package share.
    (prefix / "src" / "gtest").mkdir(parents=True)
    (prefix / "share" / "doc").mkdir()
    (prefix / "share" / "doc" / "example.launch.py").write_text("", encoding="utf-8")
    return prefix


def test_a_merged_install_tree_is_one_install_root(merged: Path) -> None:
    cfg = default_config(str(merged))
    assert [(spec.kind, spec.path) for spec in cfg.roots] == [("install", str(merged))]
    assert _profiles(cfg) == {"bringup": "alpha bringup.launch.py", "beta": "beta beta_launch.py"}
    text = scaffold.render(cfg, merged)
    payload = yaml.safe_load(text)
    assert "roots" not in payload
    assert "#   - kind: install\n#     path: .\n" in text
    assert [profile["slug"] for profile in payload["profiles"]] == ["beta", "bringup"]
    assert payload["profiles"][1]["entrypoints"] == [{"launch": "alpha bringup.launch.py"}]


def test_an_isolated_install_tree_is_one_install_root(tmp_path: Path) -> None:
    install = tmp_path / "ws" / "install"
    _installed(install / "alpha", "alpha")
    _installed(install / "beta", "beta")
    for path in (install, install.parent):
        cfg = default_config(str(path))
        assert [(spec.kind, spec.path) for spec in cfg.roots] == [("install", str(install))]
        assert _profiles(cfg) == {
            "alpha.bringup": "alpha bringup.launch.py",
            "beta.bringup": "beta bringup.launch.py",
        }


def test_a_workspace_with_only_install_reads_install(tmp_path: Path) -> None:
    _installed(tmp_path / "install", "alpha")
    cfg = default_config(str(tmp_path))
    assert [spec.kind for spec in cfg.roots] == ["install"]
    assert _profiles(cfg) == {"bringup": "alpha bringup.launch.py"}


def test_src_and_install_side_by_side_read_both(workspace: Path) -> None:
    from_src = _profiles(default_config(str(workspace)))
    _installed(workspace / "install", "binary_only")
    cfg = default_config(str(workspace))
    assert [spec.kind for spec in cfg.roots] == ["src", "install"]
    assert set(_profiles(cfg).values()) == {*from_src.values(), "binary_only bringup.launch.py"}


def test_a_package_filter_keeps_the_slugs_the_whole_tree_gives(merged: Path) -> None:
    _installed(merged, "gamma")
    whole = _profiles(default_config(str(merged)))
    only = _profiles(default_config(str(merged), ["alpha", "beta"]))
    assert only == {slug: label for slug, label in whole.items() if not label.startswith("gamma")}
    assert "alpha.bringup" in only
    with pytest.raises(UsageError, match="no launch file found in package: nowhere"):
        default_config(str(merged), ["alpha", "nowhere"])


def test_init_writes_beside_a_tree_it_may_not_write_into(merged: Path, tmp_path: Path) -> None:
    target = tmp_path / "elsewhere" / CONFIG_NAME
    target.parent.mkdir()
    argv = ["init", str(merged), "--package", "beta", "--config", str(target)]
    assert cli.main(argv) == 0
    assert not (merged / CONFIG_NAME).exists()
    payload = yaml.safe_load(target.read_text(encoding="utf-8"))
    assert payload["roots"] == [{"kind": "install", "path": str(merged)}]
    assert [profile["slug"] for profile in payload["profiles"]] == ["beta"]
    read = load_config(str(target), workspace=str(merged))
    assert [(spec.kind, spec.path) for spec in read.roots] == [("install", str(merged))]


def test_the_written_yaml_is_the_same_bytes_wherever_the_tree_sits(tmp_path: Path) -> None:
    texts = []
    for name in ("one", "two"):
        _installed(tmp_path / name / "install" / "beta", "beta")
        _installed(tmp_path / name / "install" / "alpha", "alpha", "alpha_launch.py")
        target, _cfg = scaffold.write(tmp_path / name)
        texts.append(target.read_bytes())
        assert scaffold.write(tmp_path / name, force=True)[0].read_bytes() == texts[-1]
    assert texts[0] == texts[1]


def test_packages_beside_an_install_tree_are_read_as_sources(tmp_path: Path) -> None:
    shutil.copytree(WS / "src" / "lx_bringup", tmp_path / "lx_bringup")
    (tmp_path / "build" / "stale").mkdir(parents=True)
    (tmp_path / "build" / "stale" / "package.xml").write_text("", encoding="utf-8")
    _installed(tmp_path / "install", "binary_only")
    cfg = default_config(str(tmp_path))
    assert [(spec.kind, spec.path) for spec in cfg.roots] == [
        ("src", str(tmp_path)),
        ("install", str(tmp_path / "install")),
    ]
    assert "binary_only bringup.launch.py" in _profiles(cfg).values()
    assert any(label.startswith("lx_bringup ") for label in _profiles(cfg).values())


@pytest.fixture
def read_only(merged: Path):
    merged.chmod(0o555)
    if os.access(merged, os.W_OK):
        merged.chmod(0o755)
        pytest.skip("this user writes into a read only directory anyway")
    yield merged
    merged.chmod(0o755)


def test_init_on_a_tree_it_may_not_write_into_writes_here_and_says_so(
    read_only: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys
) -> None:
    """`rosio init /opt/ros/jazzy --package nav2_bringup` ended in a PermissionError traceback."""
    here = tmp_path / "here"
    here.mkdir()
    monkeypatch.chdir(here)
    assert cli.main(["init", str(read_only), "--package", "beta"]) == 0
    out = capsys.readouterr().out
    assert not (read_only / CONFIG_NAME).exists()
    payload = yaml.safe_load((here / CONFIG_NAME).read_text(encoding="utf-8"))
    assert payload["roots"] == [{"kind": "install", "path": str(read_only)}]
    assert f"{read_only / CONFIG_NAME} is not writable" in out
    assert f"rosio extract {read_only} --config {CONFIG_NAME}" in out


def test_init_refuses_with_the_way_out_when_the_named_file_cannot_be_written(
    read_only: Path, capsys
) -> None:
    assert cli.main(["init", str(read_only), "--config", str(read_only / CONFIG_NAME)]) == 2
    err = capsys.readouterr().err
    assert "Traceback" not in err
    assert "--config" in err and "--print" in err


def test_init_on_a_tree_it_may_not_write_into_keeps_a_file_already_here(
    read_only: Path, tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys
) -> None:
    monkeypatch.chdir(tmp_path)
    (tmp_path / CONFIG_NAME).write_text("roots: []\n", encoding="utf-8")
    assert cli.main(["init", str(read_only)]) == 2
    assert "already exists" in capsys.readouterr().err


ENV_LAUNCH = """
import os
from launch import LaunchDescription


def generate_launch_description():
    model = os.environ['ROBOT_MODEL']
    lidar = os.environ['LIDAR_MODEL']
    distro = os.environ.get('ROS_DISTRO')
    quiet = os.environ.get('QUIET', 'false')
    return LaunchDescription([])
"""


def test_init_writes_a_commented_env_block_for_what_a_launch_file_reads_without_default(
    tmp_path: Path,
) -> None:
    """TurtleBot3 reads TURTLEBOT3_MODEL and LDS_MODEL with no default, and the second only
    showed up after a second extract with the first one set."""
    package = tmp_path / "src" / "bot_bringup"
    (package / "launch").mkdir(parents=True)
    (package / "package.xml").write_text(
        '<package format="3"><name>bot_bringup</name></package>', encoding="utf-8"
    )
    (package / "launch" / "robot.launch.py").write_text(ENV_LAUNCH, encoding="utf-8")
    (package / "launch" / "plain.launch.py").write_text("", encoding="utf-8")
    text = scaffold.render(default_config(str(tmp_path)), tmp_path)
    robot = text[text.index("- slug: robot") :]
    assert robot.splitlines()[4:8] == [
        "  # robot.launch.py reads these with no default, set them to what the robot starts with:",
        "  # env:",
        "  #   ROBOT_MODEL: ''",
        "  #   LIDAR_MODEL: ''",
    ]
    assert "ROS_DISTRO" not in text and "QUIET" not in text
    plain = text[text.index("- slug: plain") : text.index("- slug: robot")]
    assert "env" not in plain
    payload = yaml.safe_load(text)
    assert all("env" not in profile for profile in payload["profiles"])
    assert "env" not in payload


def test_an_install_prefix_skips_the_trees_rosio_reads_nothing_from(
    merged: Path, workspace: Path
) -> None:
    """`init --package nav2_bringup` on /opt/ros/jazzy indexed and hashed every header and
    vendored library of the distribution on every extract."""
    cfg = default_config(str(merged))
    assert cfg.ignore == scaffold_ignore()
    payload = yaml.safe_load(scaffold.render(cfg, merged))
    assert payload["ignore"] == list(scaffold_ignore())
    # a workspace that states no ignore skips its test directories
    assert load_config(None, workspace=str(workspace)).ignore == ("test", "tests")
    assert "ignore" not in yaml.safe_load(
        scaffold.render(default_config(str(workspace)), workspace)
    )


def scaffold_ignore() -> tuple[str, ...]:
    from rosio.config import INSTALL_PREFIX_IGNORE

    return INSTALL_PREFIX_IGNORE


def test_init_leaves_out_launch_files_kept_where_tests_live(workspace: Path) -> None:
    """robot_localization: nine of twelve profiles were test/test_*.launch.py."""
    package = workspace / "src" / "lx_bringup"
    plain = (
        "from launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([])\n"
    )
    names = ("test/ekf.launch.py", "tests/launch/nav.launch.py", "launch/ekf_test.launch.py")
    for relative in names:
        (package / relative).parent.mkdir(parents=True, exist_ok=True)
        (package / relative).write_text(plain, encoding="utf-8")
    _target, written = scaffold.write(workspace)
    targets = {item.target for profile in written.profiles for item in profile.entrypoints}
    assert not {"ekf.launch.py", "nav.launch.py", "ekf_test.launch.py"} & targets
    assert targets


def test_init_does_not_offer_a_launch_testing_file_as_a_profile(workspace: Path) -> None:
    """ros2_control_demos keeps a launch_testing file per example, and each became a profile
    that only ever reported its test dependencies."""
    launch = workspace / "src" / "lx_bringup" / "launch"
    (launch / "test_bringup.launch.py").write_text(
        "import launch_testing\ndef generate_test_description():\n    return None\n",
        encoding="utf-8",
    )
    (launch / "test_drive.launch.py").write_text(
        "from launch import LaunchDescription\n"
        "def generate_launch_description():\n"
        "    return LaunchDescription([])\n",
        encoding="utf-8",
    )
    _target, written = scaffold.write(workspace)
    targets = {item.target for profile in written.profiles for item in profile.entrypoints}
    assert "test_bringup.launch.py" not in targets
    assert "test_drive.launch.py" in targets
    assert "test_bringup.launch.py" not in scaffold.render(
        default_config(str(workspace)), workspace
    )


def test_init_prefixes_every_slug_once_one_stem_names_two_files(tmp_path: Path) -> None:
    install = tmp_path / "ws" / "install"
    _installed(install / "alpha", "alpha")
    _installed(install / "beta", "beta")
    _installed(install / "gamma", "gamma", "slam.launch.py")
    payload = yaml.safe_load(scaffold.render(default_config(str(install)), install))
    assert [row["slug"] for row in payload["profiles"]] == [
        "alpha.bringup",
        "beta.bringup",
        "gamma.slam",
    ]


def test_init_keeps_bare_stems_when_no_two_files_share_one(merged: Path) -> None:
    payload = yaml.safe_load(scaffold.render(default_config(str(merged)), merged))
    assert [row["slug"] for row in payload["profiles"]] == ["beta", "bringup"]


def _markup_workspace(tmp_path: Path) -> Path:
    package = tmp_path / "src" / "bot_bringup"
    (package / "launch").mkdir(parents=True)
    (package / "package.xml").write_text(
        '<package format="3"><name>bot_bringup</name></package>', encoding="utf-8"
    )
    (package / "launch" / "robot.launch.xml").write_text("<launch/>\n", encoding="utf-8")
    (package / "launch" / "plain.launch.py").write_text("", encoding="utf-8")
    return tmp_path


def test_a_markup_launch_file_is_offered_by_what_the_sourced_environment_imports(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """rosio in a virtual environment cannot import launch_xml while the walk, which runs in
    the sourced ROS environment, can, and init dropped every XML profile extract would walk."""
    from rosio.resolve import launch_exec

    workspace = _markup_workspace(tmp_path)
    asked: list[tuple] = []

    def answer(found):
        def importable(names, ros_setup=()):
            asked.append(tuple(names))
            return found

        return importable

    for found, offered in ((frozenset({"launch_xml"}), True), (frozenset(), False), (None, True)):
        monkeypatch.setattr(launch_exec, "importable", answer(found))
        slugs = [p.slug for p in scaffold.offered(default_config(str(workspace))).profiles]
        assert ("robot" in slugs) is offered, found
    assert asked and all(names == ("launch_xml",) for names in asked)


@pytest.mark.ros
def test_the_sourced_environment_answers_for_a_frontend_rosio_cannot_import(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    import importlib.util

    real = importlib.util.find_spec
    monkeypatch.setattr(
        importlib.util,
        "find_spec",
        lambda name, *rest: None if name.startswith("launch_") else real(name, *rest),
    )
    workspace = _markup_workspace(tmp_path)
    slugs = [p.slug for p in scaffold.offered(default_config(str(workspace))).profiles]
    assert "robot" in slugs


def test_init_does_not_offer_a_ros1_launch_file(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from rosio.resolve import launch_exec

    monkeypatch.setattr(launch_exec, "importable", lambda names, ros_setup=(): None)
    workspace = _markup_workspace(tmp_path)
    (workspace / "src" / "bot_bringup" / "launch" / "old.launch").write_text(
        '<launch>\n  <node pkg="bot" type="cloud" name="cloud"/>\n'
        '  <rosparam command="load" file="$(find bot)/a.yaml"/>\n</launch>\n',
        encoding="utf-8",
    )
    slugs = [p.slug for p in scaffold.offered(default_config(str(workspace))).profiles]
    assert "robot" in slugs
    assert "old" not in slugs


def test_init_reads_required_arguments_and_includes_of_xml_launch_files(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    from rosio.resolve import launch_exec

    monkeypatch.setattr(launch_exec, "importable", lambda names, ros_setup=(): None)
    workspace = _markup_workspace(tmp_path)
    launch = workspace / "src" / "bot_bringup" / "launch"
    (launch / "robot.launch.xml").write_text(
        '<launch>\n  <arg name="map_path" description="where the map is"/>\n'
        '  <arg name="rviz" default="true"/>\n'
        '  <include file="$(find-pkg-share bot_bringup)/launch/part.launch.xml">\n'
        '    <arg name="vehicle" value="$(var map_path)"/>\n  </include>\n</launch>\n',
        encoding="utf-8",
    )
    (launch / "part.launch.xml").write_text(
        '<launch>\n  <arg name="vehicle"/>\n</launch>\n', encoding="utf-8"
    )
    text = scaffold.render(default_config(str(workspace)), workspace)
    assert "robot.launch.xml declares these with no default" in text
    assert "  #     map_path: ''" in text
    assert "rviz: ''" not in text
    assert "part.launch.xml" not in text


def test_init_over_many_launch_files_names_the_ones_nothing_includes(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """Autoware: 420 profiles and no word on where a robot starts."""
    from rosio.resolve import launch_exec

    monkeypatch.setattr(launch_exec, "importable", lambda names, ros_setup=(): None)
    workspace = _markup_workspace(tmp_path)
    launch = workspace / "src" / "bot_bringup" / "launch"
    for index in range(scaffold.MANY_PROFILES):
        (launch / f"part_{index:02d}.launch.xml").write_text("<launch/>\n", encoding="utf-8")
    included = "".join(
        f'  <include file="$(find-pkg-share bot_bringup)/launch/part_{index:02d}.launch.xml"/>\n'
        for index in range(scaffold.MANY_PROFILES)
    )
    (launch / "robot.launch.xml").write_text(
        f'<launch>\n  <arg name="world"/>\n{included}</launch>\n', encoding="utf-8"
    )
    assert cli.main(["init", str(workspace)]) == 0
    said = capsys.readouterr().out
    assert "--package NAME" in said
    assert "2 files no other launch file includes: bot_bringup/plain.launch.py, " in said
    assert "bot_bringup/robot.launch.xml" in said


def test_init_comments_out_the_files_a_file_needing_no_argument_starts(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    """Each part is read by the file that includes it, so as its own profile it repeats every
    finding of that file."""
    from rosio.resolve import launch_exec

    monkeypatch.setattr(launch_exec, "importable", lambda names, ros_setup=(): None)
    workspace = _markup_workspace(tmp_path)
    launch = workspace / "src" / "bot_bringup" / "launch"
    for index in range(3):
        (launch / f"part_{index:02d}.launch.xml").write_text("<launch/>\n", encoding="utf-8")
    included = "".join(
        f'  <include file="$(find-pkg-share bot_bringup)/launch/part_{index:02d}.launch.xml"/>\n'
        for index in range(3)
    )
    (launch / "robot.launch.xml").write_text(f"<launch>\n{included}</launch>\n", encoding="utf-8")
    assert cli.main(["init", str(workspace)]) == 0
    text = (workspace / CONFIG_NAME).read_text(encoding="utf-8")
    assert text.count("# bot_bringup/robot.launch.xml includes and starts this one") == 3
    slugs = [item["slug"] for item in yaml.safe_load(text)["profiles"]]
    assert not any("part" in slug for slug in slugs)


@pytest.mark.parametrize(
    "suffix, source, hidden",
    [
        ("py", _including("part.launch.py", through_variable=True, package="bot_bringup"), True),
        ("py", "IncludeLaunchDescription(PythonLaunchDescriptionSource('part.launch.py'))", True),
        (
            "py",
            "IncludeLaunchDescription(PythonLaunchDescriptionSource('part.launch.py'), "
            "launch_arguments={}.items())",
            True,
        ),
        (
            "py",
            "IncludeLaunchDescription(PythonLaunchDescriptionSource('part.launch.py'), "
            "condition=IfCondition('false'))",
            False,
        ),
        (
            "py",
            "part = IncludeLaunchDescription(PythonLaunchDescriptionSource('part.launch.py'))\n"
            "GroupAction([GroupAction([part])], condition=IfCondition('false'))",
            False,
        ),
        (
            "py",
            "IncludeLaunchDescription(PythonLaunchDescriptionSource('part.launch.py'), "
            "launch_arguments={'use_localization': 'False'}.items())",
            False,
        ),
        (
            "py",
            "IncludeLaunchDescription(PythonLaunchDescriptionSource("
            "PathJoinSubstitution([FindPackageShare('other'), 'launch', 'part.launch.py'])))",
            False,
        ),
        (
            "py",
            "IncludeLaunchDescription(PythonLaunchDescriptionSource("
            "PathJoinSubstitution([FindPackageShare('bot_bringup'), 'launch', 'part.launch.py'])))",
            True,
        ),
        (
            "py",
            "IncludeLaunchDescription(PythonLaunchDescriptionSource("
            "os.path.join(unknown, 'part.launch.py')))",
            False,
        ),
        ("py", "PythonLaunchDescriptionSource('part.launch.py')", False),
        (
            "py",
            "def elsewhere():\n    path = 'part.launch.py'\n"
            "def generate_launch_description():\n"
            "    return IncludeLaunchDescription(PythonLaunchDescriptionSource(path))\n",
            False,
        ),
        ("xml", '<launch><include file="part.launch.py"/></launch>', True),
        ("xml", '<launch><include file="part.launch.py" if="false"/></launch>', False),
        (
            "xml",
            '<launch><group unless="true"><group><include file="part.launch.py"/>'
            "</group></group></launch>",
            False,
        ),
        (
            "xml",
            '<launch><include file="part.launch.py"><arg name="slam" value="false"/>'
            "</include></launch>",
            False,
        ),
        (
            "xml",
            '<launch><include file="$(find-pkg-share other)/launch/part.launch.py"/></launch>',
            False,
        ),
        ("yaml", "launch:\n- include:\n    file: part.launch.py\n", True),
        ("yaml", "launch:\n- include:\n    file: part.launch.py\n    unless: true\n", False),
        (
            "yaml",
            "launch:\n- group:\n    if: false\n    children:\n    - include:\n"
            "        file: part.launch.py\n",
            False,
        ),
        (
            "yaml",
            "launch:\n- include:\n    file: part.launch.py\n"
            "    arg:\n    - name: slam\n      value: 'false'\n",
            False,
        ),
        (
            "yaml",
            "launch:\n- include:\n    file: $(find-pkg-share other)/launch/part.launch.py\n",
            False,
        ),
    ],
)
def test_started_inside_requires_an_unconditional_include_without_arguments(
    tmp_path: Path, suffix: str, source: str, hidden: bool
) -> None:
    workspace = _markup_workspace(tmp_path)
    launch = workspace / "src" / "bot_bringup" / "launch"
    (launch / f"top.launch.{suffix}").write_text(source, encoding="utf-8")
    (launch / "part.launch.py").write_text("", encoding="utf-8")
    inside = scaffold.started_inside(default_config(str(workspace)).profiles)
    assert ("part" in inside) is hidden


def test_a_top_level_file_its_simulators_include_stays_offered(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    """autoware.launch.xml needs map_path and three simulator files include it, each needing
    map_path as well. It was dropped as include only, while the component it includes with
    the container name stated is the file that only runs through an include."""
    from rosio.resolve import launch_exec

    monkeypatch.setattr(launch_exec, "importable", lambda names, ros_setup=(): None)
    workspace = _markup_workspace(tmp_path)
    launch = workspace / "src" / "bot_bringup" / "launch"
    share = "$(find-pkg-share bot_bringup)/launch"
    (launch / "robot.launch.xml").write_text(
        '<launch>\n  <arg name="map_path"/>\n'
        f'  <include file="{share}/sensing.launch.xml">\n'
        '    <arg name="container" value="pointcloud_container"/>\n  </include>\n'
        f'  <include file="{share}/map.launch.xml">\n'
        '    <arg name="map_path" value="$(var map_path)"/>\n  </include>\n</launch>\n',
        encoding="utf-8",
    )
    (launch / "sensing.launch.xml").write_text(
        '<launch>\n  <arg name="container"/>\n</launch>\n', encoding="utf-8"
    )
    (launch / "map.launch.xml").write_text(
        '<launch>\n  <arg name="map_path"/>\n</launch>\n', encoding="utf-8"
    )
    (launch / "simulator.launch.xml").write_text(
        '<launch>\n  <arg name="map_path"/>\n'
        f'  <include file="{share}/robot.launch.xml">\n'
        '    <arg name="map_path" value="$(var map_path)"/>\n  </include>\n</launch>\n',
        encoding="utf-8",
    )
    slugs = [p.slug for p in scaffold.offered(default_config(str(workspace))).profiles]
    assert "robot" in slugs and "simulator" in slugs
    assert "sensing" not in slugs and "map" not in slugs


def test_an_include_spelled_with_pathlib_is_followed() -> None:
    text = (
        "from pathlib import Path\n"
        "share = Path(get_package_share_directory('bot'))\n"
        "IncludeLaunchDescription(AnyLaunchDescriptionSource(str(share / 'launch/a.launch.xml')))\n"
    )
    assert scaffold._python_includes(text) == ["$(find-pkg-share bot)/launch/a.launch.xml"]
