"""Workspace discovery: packages, roots, identity and the file selections."""

from __future__ import annotations

import json
import os
from pathlib import Path

import pytest

from rosio import ids
from rosio.config import RootSpec
from rosio.errors import ContractError, UsageError
from rosio.workspace import Workspace

PARAM_YAML = """\
demo_node:
  ros__parameters:
    rate: 20
"""

FLAT_LOOKALIKE = """\
device: tcp://203.0.113.7:28784
configure_rx: true
"""

MAP_DESCRIPTOR = """\
image: yard.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
occupied_thresh: 0.65
"""

LIST_ROOT = """\
- ros1_package_name: gps_common
  ros2_package_name: gps_msgs
"""

SETUP_PY = """\
from setuptools import setup

setup(
    name='demo_pkg',
    entry_points={
        'console_scripts': [
            'demo_node = demo_pkg.demo_node:main',
            'other_node = demo_pkg.other:main',
        ],
    },
)
"""

CMAKE = """\
add_executable(cpp_node src/cpp_node.cpp)
rclcpp_components_register_node(cpp_component
  PLUGIN "demo::Component"
  EXECUTABLE cpp_component_node)
"""


def _manifest(name: str, build_type: str = "", plugin: str = "") -> str:
    export = ""
    if build_type:
        export += f"<build_type>{build_type}</build_type>"
    if plugin:
        export += f'<nav2_core plugin="${{prefix}}/{plugin}"/>'
    return f"<package format='3'><name>{name}</name><export>{export}</export></package>"


def _write(path: Path, text: str) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")
    return path


@pytest.fixture
def bench(workspace: Path) -> Path:
    """A small two package workspace with an install tree that copies one package."""
    src = workspace / "src"
    demo = src / "demo_pkg"
    _write(demo / "package.xml", _manifest("demo_pkg", "ament_python", "demo_plugin.xml"))
    _write(demo / "setup.py", SETUP_PY)
    _write(demo / "demo_pkg" / "demo_node.py", "def main():\n    pass\n")
    _write(demo / "launch" / "demo.launch.py", "def generate_launch_description():\n    pass\n")
    _write(demo / "launch" / "extra_launch.py", "def generate_launch_description():\n    pass\n")
    _write(demo / "config" / "demo.yaml", PARAM_YAML)
    _write(demo / "config" / "flat.yaml", FLAT_LOOKALIKE)
    _write(demo / "maps" / "yard.yaml", MAP_DESCRIPTOR)
    _write(demo / "config" / "table.yaml", LIST_ROOT)
    _write(demo / "demo_plugin.xml", "<library path='demo'/>")

    cpp = src / "cpp_pkg"
    _write(cpp / "package.xml", _manifest("cpp_pkg"))
    _write(cpp / "CMakeLists.txt", CMAKE)
    _write(cpp / "src" / "cpp_node.cpp", "int main() { return 0; }\n")

    # Never walked as source.
    _write(src / "demo_pkg" / "build" / "junk.py", "raise SystemExit\n")
    _write(src / "demo_pkg" / "log" / "junk.py", "raise SystemExit\n")
    _write(src / "ignored_pkg" / "COLCON_IGNORE", "")
    _write(src / "ignored_pkg" / "package.xml", _manifest("ignored_pkg"))

    share = workspace / "install" / "demo_pkg" / "share" / "demo_pkg"
    _write(share / "package.xml", _manifest("demo_pkg", "ament_python", "demo_plugin.xml"))
    _write(share / "launch" / "demo.launch.py", "def generate_launch_description():\n    pass\n")
    _write(share / "config" / "demo.yaml", PARAM_YAML)
    _write(share / "demo_plugin.xml", "<library path='demo'/>")
    lib = workspace / "install" / "demo_pkg" / "lib" / "demo_pkg"
    _write(lib / "demo_node", "#!/usr/bin/env python3\n")
    _write(lib / "installed_only", "#!/usr/bin/env python3\n")
    return workspace


def _discover(root: Path, ignore: tuple[str, ...] = ()) -> Workspace:
    roots = [RootSpec("src", str(root / "src"))]
    if (root / "install").is_dir():
        roots.append(RootSpec("install", str(root / "install")))
    return Workspace.discover(roots, ignore)


def test_discover_needs_a_root() -> None:
    with pytest.raises(UsageError):
        Workspace.discover([])


def test_discover_refuses_a_missing_root(tmp_path: Path) -> None:
    with pytest.raises(UsageError):
        Workspace.discover([RootSpec("src", str(tmp_path / "nope"))])


def test_packages_and_build_types(bench: Path) -> None:
    ws = _discover(bench)
    assert sorted(ws.packages) == ["cpp_pkg", "demo_pkg"]
    assert ws.packages["demo_pkg"].build_type == "ament_python"
    assert ws.packages["cpp_pkg"].build_type == "ament_cmake"


def test_ignore_markers_stop_a_package(bench: Path) -> None:
    ws = _discover(bench)
    assert "ignored_pkg" not in ws.packages


def test_build_and_log_are_never_source(bench: Path) -> None:
    ws = _discover(bench)
    assert not [path for path in ws.files() if "/build/" in path or "/log/" in path]


def test_config_ignore_is_honoured(bench: Path) -> None:
    ws = _discover(bench, ignore=("*.yaml",))
    assert ws.param_files() == []
    assert ws.launch_files()


def test_executables_join_src_and_install(bench: Path) -> None:
    ws = _discover(bench)
    assert ws.packages["demo_pkg"].executables == (
        "demo_node",
        "installed_only",
        "other_node",
    )
    assert ws.packages["cpp_pkg"].executables == ("cpp_component_node", "cpp_node")


def test_plugin_manifest_prefix_resolves_to_the_share_dir(bench: Path) -> None:
    ws = _discover(bench)
    manifests = ws.packages["demo_pkg"].plugin_manifests
    assert len(manifests) == 1
    assert manifests[0].endswith("install/demo_pkg/share/demo_pkg/demo_plugin.xml")


def test_share_prefers_install_and_says_so(bench: Path) -> None:
    ws = _discover(bench)
    assert ws.share("demo_pkg") == str(bench / "install" / "demo_pkg" / "share" / "demo_pkg")
    assert ws.share_root("demo_pkg") == "install"
    assert ws.share("cpp_pkg") == str(bench / "src" / "cpp_pkg")
    assert ws.share_root("cpp_pkg") == "src"
    assert ws.share("nope") is None


def test_identity_is_the_same_in_both_trees(bench: Path) -> None:
    ws = _discover(bench)
    src = bench / "src" / "demo_pkg" / "launch" / "demo.launch.py"
    installed = bench / "install" / "demo_pkg" / "share" / "demo_pkg" / "launch" / "demo.launch.py"
    assert ws.file_id(str(src)) == ws.file_id(str(installed))
    assert ws.file_id(str(src)) == ids.build_file("demo_pkg", "launch/demo.launch.py")
    assert ws.root_of(str(src)) == "src"
    assert ws.root_of(str(installed)) == "install"


def test_package_of_and_rel(bench: Path) -> None:
    ws = _discover(bench)
    src = bench / "src" / "demo_pkg" / "config" / "demo.yaml"
    assert ws.package_of(str(src)) == "demo_pkg"
    assert ws.rel(str(src)) == ("demo_pkg", "config/demo.yaml")


def test_a_file_outside_every_package_is_repo_relative(bench: Path) -> None:
    loose = _write(bench / "src" / "notes" / "loose.py", "x = 1\n")
    ws = _discover(bench)
    package, relative = ws.rel(str(loose))
    assert package is None
    assert relative.endswith("notes/loose.py")
    assert ids.parse_file(ws.file_id(str(loose))).package == ids.REPO_PKG


def test_a_path_outside_every_root_is_refused(bench: Path) -> None:
    ws = _discover(bench)
    with pytest.raises(ContractError):
        ws.file_id("/tmp/merged_params.yaml")


def test_abspath_round_trips(bench: Path) -> None:
    ws = _discover(bench)
    src = bench / "src" / "cpp_pkg" / "src" / "cpp_node.cpp"
    assert ws.abspath(ws.file_id(str(src))) == str(src)


def test_site_and_source(bench: Path) -> None:
    ws = _discover(bench)
    src = bench / "src" / "demo_pkg" / "config" / "demo.yaml"
    site = ws.site(str(src), "demo_node.ros__parameters.rate", 3)
    assert site.path == str(src)
    assert site.root == "src"
    source = ws.source(site)
    assert source.file == ids.build_file("demo_pkg", "config/demo.yaml")
    assert source.locator == "demo_node.ros__parameters.rate"
    assert source.line_hint == 3


# ---------------------------------------------------------------------------
# citation through the package share


def _installed(bench: Path, relative: str) -> Path:
    return bench / "install" / "demo_pkg" / "share" / "demo_pkg" / relative


def test_a_share_read_of_an_identical_copy_is_cited_as_src(bench: Path) -> None:
    """The editable file is the src copy, and equal bytes make the two interchangeable."""
    ws = _discover(bench)
    installed = _installed(bench, "config/demo.yaml")
    cited, root = ws.cite(str(installed))
    assert cited == str(bench / "src" / "demo_pkg" / "config" / "demo.yaml")
    assert root == "src"
    read = ws.share_reads()[0]
    assert read.file_id == ids.build_file("demo_pkg", "config/demo.yaml")
    assert read.identical


def test_a_share_read_of_a_differing_copy_keeps_the_tree_it_read(bench: Path) -> None:
    """Honest root: the values came out of the install tree and the trees disagree."""
    installed = _installed(bench, "config/demo.yaml")
    installed.write_text(PARAM_YAML.replace("20", "10"), encoding="utf-8")
    ws = _discover(bench)
    cited, root = ws.cite(str(installed))
    assert cited == str(installed)
    assert root == "install"
    read = ws.share_reads()[0]
    assert not read.identical
    assert read.src_path == str(bench / "src" / "demo_pkg" / "config" / "demo.yaml")


def test_a_share_read_reaches_the_evidence_row(bench: Path) -> None:
    ws = _discover(bench)
    installed = _installed(bench, "config/demo.yaml")
    source = ws.source(ws.site(str(installed), "demo_node.ros__parameters.rate", 3))
    assert source.file == ids.build_file("demo_pkg", "config/demo.yaml")
    assert source.root == "src"
    assert source.locator == "demo_node.ros__parameters.rate"


def test_an_install_only_file_is_still_cited_as_install(bench: Path) -> None:
    ws = _discover(bench)
    installed = bench / "install" / "demo_pkg" / "lib" / "demo_pkg" / "installed_only"
    assert ws.src_twin(str(installed)) is None
    assert ws.cite(str(installed)) == (str(installed), "install")
    assert ws.share_reads() == []


def test_a_src_path_is_never_redirected(bench: Path) -> None:
    ws = _discover(bench)
    src = bench / "src" / "demo_pkg" / "config" / "demo.yaml"
    assert ws.cite(str(src)) == (str(src), "src")
    assert ws.src_twin(str(src)) is None
    assert ws.share_reads() == []


def test_a_twin_that_is_not_the_source_of_the_installed_copy(bench: Path) -> None:
    """A package that installs a message out of msg_ros2/ keeps a different file of that
    name under msg/, so a shared relative path is not provenance. The citation must stay on
    the tree that was read."""
    _write(bench / "src" / "demo_pkg" / "msg" / "Block.msg", "# ros 1 spelling\nuint8 sync\n")
    _write(bench / "src" / "demo_pkg" / "msg_ros2" / "msg" / "Block.msg", "uint8 sync 36\n")
    installed = _write(_installed(bench, "msg/Block.msg"), "uint8 sync 36\n")
    ws = _discover(bench)

    cited, root = ws.cite(str(installed))
    assert cited == str(installed)
    assert root == "install"
    assert not ws.share_reads()[0].identical


def test_share_reads_are_sorted_and_stable(bench: Path) -> None:
    ws = _discover(bench)
    for relative in ("launch/demo.launch.py", "config/demo.yaml", "package.xml"):
        ws.cite(str(_installed(bench, relative)))
        ws.cite(str(_installed(bench, relative)))
    reads = ws.share_reads()
    assert [read.file_id for read in reads] == sorted(read.file_id for read in reads)
    assert len(reads) == 3


def test_param_files_are_shape_based(bench: Path) -> None:
    ws = _discover(bench)
    found = [ws.file_id(path) for path in ws.param_files()]
    assert found == [ids.build_file("demo_pkg", "config/demo.yaml")]


def test_launch_files_cover_both_spellings(bench: Path) -> None:
    ws = _discover(bench)
    found = [ws.file_id(path) for path in ws.launch_files()]
    assert found == [
        ids.build_file("demo_pkg", "launch/demo.launch.py"),
        ids.build_file("demo_pkg", "launch/extra_launch.py"),
    ]


def test_selections_prefer_src_over_a_stale_copy(bench: Path) -> None:
    ws = _discover(bench)
    for path in ws.launch_files() + ws.param_files():
        assert ws.root_of(path) == "src"


def test_source_files_by_extension(bench: Path) -> None:
    ws = _discover(bench)
    assert [ws.file_id(path) for path in ws.source_files(["cpp"])] == [
        ids.build_file("cpp_pkg", "src/cpp_node.cpp")
    ]
    assert ws.source_files([".cpp"]) == ws.source_files(["cpp"])


def test_selections_are_sorted_and_stable(bench: Path) -> None:
    first = _discover(bench)
    second = _discover(bench)
    assert first.param_files() == second.param_files()
    assert first.launch_files() == second.launch_files()
    assert first.root_hashes() == second.root_hashes()


def test_an_overlay_package_wins_over_a_base_copy(tmp_path: Path) -> None:
    """Configured root order decides which copy of a duplicated package is read, so a
    workspace overlaid on a base does not report the base as the system it runs."""
    base = tmp_path / "aa_base" / "src" / "demo_pkg"
    overlay = tmp_path / "zz_overlay" / "src" / "demo_pkg"
    for root, marker in ((base, "base"), (overlay, "overlay")):
        _write(root / "package.xml", _manifest("demo_pkg", "ament_python"))
        _write(root / "setup.py", SETUP_PY.replace("demo_node:main", f"{marker}:main"))
        _write(root / "launch" / f"{marker}.launch.py", "\n")
    ws = Workspace.discover(
        [
            RootSpec("src", str(tmp_path / "zz_overlay" / "src")),
            RootSpec("src", str(tmp_path / "aa_base" / "src")),
        ]
    )
    assert ws.packages["demo_pkg"].src_path == str(overlay)
    # The base copy sorts first by absolute path, which is exactly what must not decide.
    assert str(base) < str(overlay)
    flipped = Workspace.discover(
        [
            RootSpec("src", str(tmp_path / "aa_base" / "src")),
            RootSpec("src", str(tmp_path / "zz_overlay" / "src")),
        ]
    )
    assert flipped.packages["demo_pkg"].src_path == str(base)


def test_two_roots_of_one_kind_keep_separate_hashes(tmp_path: Path) -> None:
    """A shared basename must not collapse two root labels into one, or one hash silently
    replaces the other and a changed tree reports no change."""
    left = tmp_path / "opt" / "alpha" / "src"
    right = tmp_path / "srv" / "beta" / "src"
    _write(left / "demo_pkg" / "package.xml", _manifest("demo_pkg"))
    _write(right / "other_pkg" / "package.xml", _manifest("other_pkg"))
    ws = Workspace.discover([RootSpec("src", str(left)), RootSpec("src", str(right))])
    hashes = ws.root_hashes()
    assert sorted(hashes) == ["src:alpha/src", "src:beta/src"]
    assert len(set(hashes.values())) == 2


def test_hostile_yaml_is_not_a_parameter_file_and_does_not_escape(tmp_path: Path) -> None:
    """Deep nesting comes out of the parser as RecursionError, not as a YAMLError, so a
    discovery pass that catches only YAMLError takes the whole run down."""
    src = tmp_path / "src" / "demo_pkg"
    _write(src / "package.xml", _manifest("demo_pkg"))
    _write(src / "config" / "deep.yaml", "a: " + "[" * 4000 + "]" * 4000)
    _write(src / "config" / "unhashable.yaml", "? [1, 2]\n: value\n")
    _write(src / "config" / "good.yaml", PARAM_YAML)
    ws = Workspace.discover([RootSpec("src", str(tmp_path / "src"))])
    assert [Path(path).name for path in ws.param_files()] == ["good.yaml"]


def _built_at(root: Path) -> Path:
    """A checkout that has been built: the build tool writes the prefix of this machine into
    its own setup files, which is what makes a built tree machine specific by content."""
    (root / ".git").mkdir(exist_ok=True)
    prefix = root / "install"
    _write(
        prefix / "setup.bash",
        f'COLCON_CURRENT_PREFIX="{prefix}"\nexport AMENT_PREFIX_PATH="{prefix}"\n',
    )
    _write(
        prefix / "demo_pkg" / "share" / "demo_pkg" / "package.dsv",
        f"prepend-non-duplicate;AMENT_PREFIX_PATH;{prefix}/demo_pkg\n",
    )
    _write(root / "src" / "demo_pkg" / "config" / "built.yaml", PARAM_YAML)
    return root


def test_a_built_checkout_at_two_paths_hashes_its_roots_the_same(
    workspace: Path, bench: Path, tmp_path: Path
) -> None:
    """The blocker for a team where everybody builds. Colcon writes the absolute prefix of
    the machine it ran on into its own setup files, those files sit inside a root, and the
    root hash therefore separated two clones of identical sources."""
    import shutil

    left = _built_at(bench)
    right = tmp_path / "far" / "away" / "under" / "a" / "much" / "longer" / "path" / "ws"
    shutil.copytree(left, right)
    # the copy states its own prefix, the way a second machine's build would
    _write(
        right / "install" / "setup.bash",
        f'COLCON_CURRENT_PREFIX="{right / "install"}"\n'
        f'export AMENT_PREFIX_PATH="{right / "install"}"\n',
    )
    _write(
        right / "install" / "demo_pkg" / "share" / "demo_pkg" / "package.dsv",
        f"prepend-non-duplicate;AMENT_PREFIX_PATH;{right / 'install'}/demo_pkg\n",
    )

    assert _discover(left).root_hashes() == _discover(right).root_hashes()


def test_a_setup_file_that_changed_for_a_reason_still_moves_the_hash(bench: Path) -> None:
    """The gate has to keep working. Only the substring that says where the tree is mounted
    is normalised, so an overlay somewhere else, a package added and a hand written line are
    all still input differences."""
    built = _built_at(bench)
    setup = built / "install" / "setup.bash"
    before = _discover(built).root_hashes()["install"]

    _write(setup, setup.read_text(encoding="utf-8") + 'export AMENT_PREFIX_PATH="/opt/other"\n')
    assert _discover(built).root_hashes()["install"] != before

    _write(setup, f'COLCON_CURRENT_PREFIX="{built / "install"}"\n')
    assert _discover(built).root_hashes()["install"] != before

    _write(built / "install" / "demo_pkg" / "share" / "demo_pkg" / "extra.yaml", PARAM_YAML)
    assert _discover(built).root_hashes()["install"] != before


def test_the_marks_name_every_root_and_the_checkout(bench: Path) -> None:
    marks = _discover(bench).content_marks()
    assert [mark for _base, mark in marks].count(b"\x00@src\x00") >= 1
    assert [mark for _base, mark in marks].count(b"\x00@install\x00") >= 1
    # longest base first, or a root nested in the checkout loses to the checkout
    lengths = [len(base) for base, _mark in marks]
    assert lengths == sorted(lengths, reverse=True)


def test_root_hashes_follow_content(bench: Path) -> None:
    before = _discover(bench).root_hashes()
    assert sorted(before) == ["install", "src"]
    _write(bench / "src" / "demo_pkg" / "config" / "demo.yaml", PARAM_YAML.replace("20", "30"))
    after = _discover(bench).root_hashes()
    assert after["src"] != before["src"]
    assert after["install"] == before["install"]


# ---------------------------------------------------------------------------
# the reference workspace
#
# What this workspace holds is not written down in this repository. Point
# ROSIO_REFERENCE_EXPECT at a json file describing the workspace named by
# ROSIO_REFERENCE_WS, and these tests run against it. Without either variable they skip,
# so a stranger's suite is green and nothing about anybody's private tree ships here.
# docs/reference-expectations.md holds the schema of that file and an example.


REFERENCE_EXPECT_ENV = "ROSIO_REFERENCE_EXPECT"


@pytest.fixture(scope="module")
def reference_expect() -> dict:
    configured = os.environ.get(REFERENCE_EXPECT_ENV)
    if not configured:
        pytest.skip(f"{REFERENCE_EXPECT_ENV} is not set, see docs/reference-expectations.md")
    path = Path(configured)
    if not path.is_file():
        pytest.skip(f"reference expectations not present at {path}")
    return json.loads(path.read_text(encoding="utf-8"))


@pytest.fixture(scope="module")
def reference_workspace(reference_ws: Path) -> Workspace:
    return Workspace.discover(
        [
            RootSpec("src", str(reference_ws / "src")),
            RootSpec("install", str(reference_ws / "install")),
        ]
    )


@pytest.mark.slow
def test_reference_packages(reference_workspace: Workspace, reference_expect: dict) -> None:
    assert len(reference_workspace.packages) == reference_expect["package_count"]
    for name, build_type in reference_expect["build_types"].items():
        assert reference_workspace.packages[name].build_type == build_type, name


@pytest.mark.slow
def test_reference_executables(reference_workspace: Workspace, reference_expect: dict) -> None:
    for name, executables in reference_expect["executables_contain"].items():
        found = reference_workspace.packages[name].executables
        for executable in executables:
            assert executable in found, (name, executable)
    for name, executables in reference_expect["executables_exact"].items():
        assert reference_workspace.packages[name].executables == tuple(executables), name


@pytest.mark.slow
def test_reference_plugin_manifests(reference_workspace: Workspace, reference_expect: dict) -> None:
    for name, expected in reference_expect["plugin_manifests"].items():
        manifests = reference_workspace.packages[name].plugin_manifests
        assert [Path(path).name for path in manifests] == expected, name
        assert all(Path(path).is_file() for path in manifests)


@pytest.mark.slow
def test_reference_launch_and_param_counts(
    reference_workspace: Workspace, reference_expect: dict
) -> None:
    launch = reference_workspace.launch_files()
    params = reference_workspace.param_files()
    assert len(launch) == reference_expect["launch_file_count"]
    legacy = [path for path in launch if path.endswith(".launch")]
    assert len(legacy) == reference_expect["legacy_launch_count"]
    assert len(params) == reference_expect["param_file_count"]
    names = {Path(path).name for path in params}
    assert set(reference_expect["param_file_names"]) <= names
    # Shape based: these look like parameter files or are yaml, and are neither.
    assert not names & set(reference_expect["not_param_file_names"])


@pytest.mark.slow
def test_reference_share_reads_land_on_src(
    reference_ws: Path, reference_workspace: Workspace, reference_expect: dict
) -> None:
    """The launch files resolve their configs through the package share, and this build is
    current, so every one of them is cited on the editable file."""
    package = reference_expect["share_package"]
    share = reference_ws / "install" / package / "share" / package / "config"
    for name in reference_expect["share_config_names"]:
        cited, root = reference_workspace.cite(str(share / name))
        assert root == "src"
        assert cited == str(reference_ws / "src" / package / "config" / name)
    assert all(read.identical for read in reference_workspace.share_reads())


@pytest.mark.slow
def test_reference_repo_root_and_ids(
    reference_workspace: Workspace, reference_expect: dict
) -> None:
    assert Path(reference_workspace.repo_root).name == reference_expect["repo_dir_name"]
    package = reference_expect["launch_file_package"]
    relative = reference_expect["launch_file_relpath"]
    launch = next(path for path in reference_workspace.launch_files() if path.endswith(relative))
    assert reference_workspace.file_id(launch) == ids.build_file(package, relative)
    assert reference_workspace.root_of(launch) == "src"


# ---------------------------------------------------------------------------
# root membership by real path


def test_a_root_reached_through_a_symlink_still_resolves_inside(bench: Path, tmp_path: Path):
    """A launch file that self locates with os.path.realpath names the canonical path of a
    root configured through a symlink. Comparing the text puts its own config directory
    outside the workspace and writes a machine path into compared data."""
    alias = tmp_path / "alias"
    alias.symlink_to(bench)
    ws = Workspace.discover(
        [
            RootSpec("src", str(alias / "src")),
            RootSpec("install", str(alias / "install")),
        ]
    )
    canonical = str(bench / "src" / "demo_pkg" / "config" / "demo.yaml")
    assert ws.inside(canonical)
    assert ws.aliased(canonical)
    assert ws.localise(canonical) == str(alias / "src" / "demo_pkg" / "config" / "demo.yaml")
    assert ws.file_id(canonical) == ids.build_file("demo_pkg", "config/demo.yaml")
    assert ws.root_of(canonical) == "src"
    assert ws.rel(canonical) == ("demo_pkg", "config/demo.yaml")


def test_a_path_outside_every_root_is_left_alone(bench: Path, tmp_path: Path):
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    outside = str(tmp_path.parent / "elsewhere" / "params.yaml")
    assert not ws.inside(outside)
    assert not ws.aliased(outside)
    assert ws.localise(outside) == outside


def test_a_digest_token_names_a_file_and_never_a_mount(bench: Path, tmp_path: Path):
    alias = tmp_path / "alias"
    alias.symlink_to(bench)
    plain = Workspace.discover([RootSpec("src", str(bench / "src"))])
    aliased = Workspace.discover([RootSpec("src", str(alias / "src"))])
    canonical = str(bench / "src" / "demo_pkg" / "config" / "demo.yaml")
    through_alias = str(alias / "src" / "demo_pkg" / "config" / "demo.yaml")
    assert plain.token(canonical) == aliased.token(through_alias)
    assert plain.token(canonical) == ids.build_file("demo_pkg", "config/demo.yaml")
    assert plain.token("cmd_vel_nav") == "cmd_vel_nav"
    assert plain.token("/opt/ros/humble/setup.bash") == "/opt/ros/humble/setup.bash"


def test_a_file_outside_the_checkout_is_stated_by_its_name_and_its_content(
    bench: Path, tmp_path: Path
):
    """Two boxes hold the ROS installation at two prefixes and the sources are the same, so
    an input digest over the raw path gave them two run ids and two atlases. The identity
    they can match is the name and the content, with the prefix taken out first the way
    `content_marks` takes it out of a root."""
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    left = tmp_path / "opt" / "ros" / "humble"
    right = tmp_path / "home" / "someone" / "ros_humble"
    for prefix in (left, right):
        prefix.mkdir(parents=True)
        (prefix / "setup.bash").write_text("AMENT_SHELL=bash\n", encoding="utf-8")

    stated = ws.stated([str(left / "setup.bash"), str(right / "setup.bash")])
    assert stated[0] != stated[1]

    marks = ws.stated_by_content([str(left / "setup.bash"), str(right / "setup.bash")])
    assert marks[0] == marks[1]
    assert marks[0].startswith("@outside/setup.bash#")
    assert str(tmp_path) not in marks[0]


def test_a_value_names_a_root_by_its_label_and_keeps_its_text(bench: Path, tmp_path: Path):
    """A value is what a node receives, so only the mount point changes: the text around it,
    and a path outside every root, stay as they were."""
    ws = Workspace.discover(
        [RootSpec("src", str(bench / "src")), RootSpec("install", str(bench / "install"))]
    )
    ros = tmp_path / "opt" / "ros" / "jazzy"
    banner = f"<!-- autogenerated by xacro from {bench}/src/demo_pkg/urdf/bot.xacro -->"
    value = {
        "robot_description": banner,
        "maps": [f"{bench}/install/share/demo_pkg/map.yaml", "/srv/maps/map.yaml"],
        "search": f"searching: ['{bench}/install/demo_pkg', '{ros}']",
        "rate": 20,
        "near": [f"{bench}/src_backup/params.yaml", f"{bench}/srcfoo", f"{ros}_custom/x.yaml"],
    }
    assert ws.stated_value(value, (str(ros / "setup.bash"),)) == {
        "robot_description": "<!-- autogenerated by xacro from @src/demo_pkg/urdf/bot.xacro -->",
        "maps": ["@install/share/demo_pkg/map.yaml", "/srv/maps/map.yaml"],
        "search": "searching: ['@install/demo_pkg', '@outside']",
        "rate": 20,
        "near": [
            "@checkout/src_backup/params.yaml",
            "@checkout/srcfoo",
            "@checkout/opt/ros/jazzy_custom/x.yaml",
        ],
    }
    root_setup = str(bench / "install" / "setup.bash")
    assert ws.stated_value(str(bench / "install"), (root_setup,)) == "@install"


def test_a_message_states_every_path_it_names_and_no_ros_name(bench: Path, tmp_path: Path):
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    ros = tmp_path / "opt" / "ros" / "jazzy"
    text = (
        f"remapping /cmd_vel to /robot/cmd_vel failed in '/somewhere/else/x.yaml', "
        f"/opt/ros/jazzy/lib/x_node, {ros}/share/nav2_bringup/launch/nav.py:12, "
        f"{bench}/src/demo_pkg/config/demo.yaml, package://demo_pkg/meshes/base.stl, "
        "file:///usr/share/x/base.dae and /home/robot."
    )
    assert ws.stated_message(text, (str(ros / "setup.bash"),)) == (
        "remapping /cmd_vel to /robot/cmd_vel failed in '@outside/x.yaml', "
        "@outside/x_node, @outside/share/nav2_bringup/launch/nav.py:12, "
        "@src/demo_pkg/config/demo.yaml, package://demo_pkg/meshes/base.stl, "
        "file://@outside/base.dae and @outside/robot."
    )


def test_a_message_states_a_directory_python_quotes_under_any_top(bench: Path, tmp_path: Path):
    """An overlay that is not a root, `/overlay_ws` in a container image, is named in the
    search list of every missing package. Python quotes a path it raises on and every item
    of a list, so a quoted token of two segments is a path whatever its top, while a ROS
    name outside quotes, or quoted alone in prose, keeps its spelling."""
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    setup = (str(tmp_path / "opt" / "ros" / "jazzy" / "setup.bash"),)
    ros = str(tmp_path / "opt" / "ros" / "jazzy")
    cases = {
        f"package 'nav2_bringup' not found, searching: ['/overlay_ws/install/nav2_bringup', "
        f"'{ros}']": "package 'nav2_bringup' not found, searching: ['@outside/nav2_bringup', "
        "'@outside']",
        "[Errno 2] No such file or directory: '/data/maps/office'": (
            "[Errno 2] No such file or directory: '@outside/office'"
        ),
        "cannot read /my-ws/install/maps": "cannot read @outside/maps",
        "remap /robot/cmd_vel to '/cmd_vel' for topic '/robot/odom'": (
            "remap /robot/cmd_vel to '/cmd_vel' for topic '/robot/odom'"
        ),
        "names: ['/cmd_vel']": "names: ['/cmd_vel']",
    }
    for text, stated in cases.items():
        assert ws.stated_message(text, setup) == stated


def test_a_setup_prefix_is_replaced_only_where_a_path_starts(bench: Path, tmp_path: Path):
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    ros = str(tmp_path / "opt" / "ros" / "jazzy")
    setup = (f"{ros}/setup.bash",)
    assert ws.stated_value(f"/data{ros}/share/x/y.yaml", setup) == f"/data{ros}/share/x/y.yaml"
    assert ws.stated_path(f"/data{ros}/share/x/y.yaml", setup) == "@outside/y.yaml"
    assert ws.stated_value(f"file://{ros}/share/x/y.dae", setup) == "file://@outside/share/x/y.dae"
    assert ws.stated_value(f"['{ros}/share', {ros}]", setup) == "['@outside/share', @outside]"


def test_the_content_of_a_file_outside_the_checkout_still_moves_its_mark(
    bench: Path, tmp_path: Path
):
    """The gate has to keep working: only the substring naming the mount point is
    normalised, so a setup file that gained a package or a hand written line still moves."""
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    outside = tmp_path / "opt" / "ros" / "humble"
    outside.mkdir(parents=True)
    setup = outside / "setup.bash"

    setup.write_text("AMENT_SHELL=bash\n", encoding="utf-8")
    before = ws.stated_by_content([str(setup)])[0]
    setup.write_text("AMENT_SHELL=bash\nexport EXTRA=1\n", encoding="utf-8")
    after = ws.stated_by_content([str(setup)])[0]
    assert before != after

    setup.unlink()
    assert ws.stated_by_content([str(setup)]) == ["@outside/setup.bash#absent"]


def test_a_setup_file_inside_the_checkout_still_states_its_place(bench: Path):
    """`stated_by_content` extends `stated`, it does not replace its answer."""
    (bench / ".git").mkdir(exist_ok=True)
    ws = Workspace.discover(
        [RootSpec("src", str(bench / "src")), RootSpec("install", str(bench / "install"))]
    )
    setup = bench / "install" / "setup.bash"
    setup.write_text("export FOO=1\n", encoding="utf-8")
    assert ws.stated_by_content([str(setup)]) == ["install/setup.bash"]
    assert ws.stated_by_content([str(setup)]) == ws.stated([str(setup)])


def _write_undecodable(directory: Path) -> None:
    """A name a filesystem allows and no decoder accepts, which os.walk hands back as lone
    surrogates."""
    name = os.fsdecode(os.fsencode(str(directory)) + b"/bad\xff.py")
    Path(name).write_bytes(b"def main():\n    pass\n")


def test_a_file_name_that_is_not_utf8_is_named_and_costs_nothing_else(bench: Path):
    """One such name anywhere under a root used to end rosio with a raw UnicodeEncodeError
    at root_hashes and write no atlas at all."""
    _write_undecodable(bench / "src" / "demo_pkg" / "demo_pkg")
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    assert ws.root_hashes()["src"]
    assert all("\udcff" not in path for path in ws.files())
    rows = ws.undecodable()
    assert [row["path"] for row in rows] == ["demo_pkg/demo_pkg/bad%FF.py"]
    assert rows[0]["phase"] == "walk"
    assert rows[0]["exc_type"] == "UnicodeDecodeError"
    assert ws.undecodable_owners() == [(ids.build_pkg("demo_pkg"), "demo_pkg/demo_pkg/bad%FF.py")]


def test_a_workspace_with_only_nameable_files_says_nothing(bench: Path):
    ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    assert ws.undecodable() == []
    assert ws.undecodable_owners() == []


def test_the_root_hash_still_covers_a_name_the_index_refused(bench: Path):
    """Dropping such a file from the digest would let a root change with no run saying so."""
    before = Workspace.discover([RootSpec("src", str(bench / "src"))]).root_hashes()
    _write_undecodable(bench / "src" / "demo_pkg" / "demo_pkg")
    after = Workspace.discover([RootSpec("src", str(bench / "src"))]).root_hashes()
    assert before != after


# ---------------------------------------------------------------------------
# a build file this process cannot open


@pytest.mark.skipif(os.geteuid() == 0, reason="root opens a file with mode 0")
def test_a_build_file_this_process_cannot_open_does_not_end_discovery(bench: Path) -> None:
    """One unreadable CMakeLists.txt used to raise PermissionError out of `discover`, which
    ended the whole extraction with a traceback and exit 1, before any collector ran and
    with no atlas written. Discovery now stands and the file is left for the collector that
    reads it to refuse."""
    cmake = bench / "src" / "cpp_pkg" / "CMakeLists.txt"
    cmake.chmod(0o000)
    try:
        ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
    finally:
        cmake.chmod(0o644)
    assert str(cmake) in ws.files()
    assert ws.packages["cpp_pkg"].executables == ()
    assert ws.packages["demo_pkg"].executables == ("demo_node", "other_node")
    assert ws.root_hashes()["src"]


@pytest.mark.skipif(os.geteuid() == 0, reason="root opens a file with mode 0")
def test_a_parameters_file_this_process_cannot_open_is_not_a_parameters_file(
    bench: Path,
) -> None:
    """The yaml predicate reads the file too, and the same refusal reaches it as a
    CollectorError it already handles."""
    yaml_path = bench / "src" / "demo_pkg" / "config" / "demo.yaml"
    yaml_path.chmod(0o000)
    try:
        ws = Workspace.discover([RootSpec("src", str(bench / "src"))])
        selected = ws.param_files()
    finally:
        yaml_path.chmod(0o644)
    assert str(yaml_path) in ws.files()
    assert str(yaml_path) not in selected
