"""The extract orchestration: one atlas, one order, one file.

The fixture workspace is used end to end here, so this suite exercises what the CLI runs and
not a stand in. Two properties are asserted the hard way, by running twice: the collector
order does not reach the output, and neither does anything else that changes between runs.

Three properties are asserted because getting them wrong is silent. The default selection is
the committed order and not the registry, or an atlas depends on which module was imported
first. A layer is present only when a collector for it produced a fact, or a collector that
raised leaves a layer claiming a truth it does not hold. And the rules run here, or the
product of the tool is a file nobody reads.
"""

from __future__ import annotations

import json
from pathlib import Path

import pytest

from rosio import extract, ids, model, schema
from rosio.collect import REGISTRY, CollectContext
from rosio.config import Config, Entrypoint, ProfileSpec, RootSpec
from rosio.errors import CollectorError, ContractError, UsageError
from rosio.workspace import Workspace

WS = Path(__file__).parent / "fixtures" / "ws_launch"
CODE_WS = Path(__file__).parent / "fixtures" / "ws_code"

LAYER = model.Layer.DECLARED.value
CODED = model.Layer.CODED.value
OBSERVED = model.Layer.OBSERVED.value
RECORDED = model.Layer.RECORDED.value

PROBE = "probe"
BAG = "bag"

OPT_IN_PROBE = (PROBE,)
"""What a caller states to reach the live probe. `extract` never selects it and refuses to
be told to, because this bench box is attached to a vehicle."""


def _cpp_extra() -> bool:
    from rosio.collect import cpp_code

    try:
        cpp_code.load_parser()
    except CollectorError:
        return False
    return True


needs_cpp = pytest.mark.skipif(not _cpp_extra(), reason="the cpp extra is not installed")

PROFILES = (
    ProfileSpec(
        slug="plain",
        label="one plain node",
        entrypoints=(Entrypoint(verb="launch", pkg="lx_bringup", target="plain_node.launch.py"),),
    ),
    ProfileSpec(
        slug="stack",
        label="the nav stack composition",
        entrypoints=(
            Entrypoint(verb="launch", pkg="lx_bringup", target="nav_stack.launch.py"),
            Entrypoint(verb="launch", pkg="lx_sensors", target="sensors_only.launch.py"),
        ),
    ),
)


@pytest.fixture(scope="module")
def ws() -> Workspace:
    return Workspace.discover(
        (
            RootSpec(kind="src", path=str(WS / "src")),
            RootSpec(kind="install", path=str(WS / "install")),
        )
    )


def _config(out_dir: Path, ws: Workspace, profiles=PROFILES) -> Config:
    return Config(
        roots=tuple(RootSpec(kind=root.kind, path=root.path) for root in ws.roots),
        profiles=tuple(profiles),
        out_dir=str(out_dir),
        deterministic=True,
    )


# ---------------------------------------------------------------------------
# the whole thing


def test_extract_writes_a_valid_atlas(tmp_path: Path, ws: Workspace):
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws)

    assert schema.validate(atlas.to_json()) == []
    target = extract.out_path(cfg)
    assert target.is_file()
    reloaded = model.Atlas.loads(target.read_text(encoding="utf-8"))
    assert reloaded.canonical_bytes() == atlas.canonical_bytes()
    assert atlas.entities, "an empty atlas is not an extraction"


def test_every_atlas_holds_the_workspace_entity(tmp_path: Path, ws: Workspace):
    """A finding about the whole run is about this entity, so adding a profile moves no id."""
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    assert ids.build_pkg(ids.REPO_PKG) in atlas.entities


def test_the_file_is_written_through_the_pinned_encoder(tmp_path: Path, ws: Workspace):
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws)
    text = extract.out_path(cfg).read_text(encoding="utf-8")
    assert text == model.dumps(atlas.to_json())
    assert text.endswith("\n")


def test_every_declared_profile_gets_a_profile_entity_and_a_run(tmp_path: Path, ws: Workspace):
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws)

    slugs = {ids.parse_profile(node.id) for node in atlas.by_kind("profile")}
    assert slugs == {spec.slug for spec in PROFILES}
    for slug in slugs:
        runs = [
            node
            for node in atlas.by_kind("run")
            if node.attrs["profile"] == ids.build_profile(slug)
        ]
        assert runs, f"no run was registered for profile {slug}"
        assert all(node.attrs["layer"] == LAYER for node in runs)
    assert atlas.layers[LAYER].present
    assert atlas.layers[LAYER].runs == sorted(atlas.layers[LAYER].runs)


def test_workspace_wide_facts_carry_no_profile(tmp_path: Path, ws: Workspace):
    """An interface definition holds whatever runs, so it must not claim a profile."""
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws, collectors=["interfaces", "plugins", "urdf", "entrypoints"])
    facts = [fact for node in atlas.entities.values() for fact in node.facts]
    assert facts, "the repository collectors said nothing at all"
    assert {fact.profile for fact in facts} == {model.ANY_PROFILE}


def test_capabilities_are_declared_and_sorted(tmp_path: Path, ws: Workspace):
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    assert atlas.capabilities == sorted(extract.CAPABILITIES)
    assert "layer.declared" in atlas.capabilities


# ---------------------------------------------------------------------------
# determinism


def test_collector_order_does_not_reach_the_output(tmp_path: Path, ws: Workspace):
    """Over the committed selection, not the registry: a collector another module registered
    on import would otherwise join this run and make the property depend on the suite."""
    names = extract.default_collectors()
    first = extract.run(_config(tmp_path / "a", ws), ws, collectors=names)
    second = extract.run(_config(tmp_path / "b", ws), ws, collectors=list(reversed(names)))
    assert first.canonical_bytes() == second.canonical_bytes()


def test_two_runs_over_unchanged_inputs_are_byte_identical(tmp_path: Path, ws: Workspace):
    first = extract.run(_config(tmp_path / "a", ws), ws)
    second = extract.run(_config(tmp_path / "b", ws), ws)
    assert first.canonical_bytes() == second.canonical_bytes()


def test_the_generation_block_is_outside_the_compared_bytes(tmp_path: Path, ws: Workspace):
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws)
    assert atlas.generated["profiles"] == ["plain", "stack"]
    assert "at" not in atlas.generated, "a deterministic run must not write a clock"
    assert b'"generated"' in atlas.to_bytes()
    assert b'"generated"' not in atlas.canonical_bytes()


def test_the_generation_block_states_a_root_by_identity_and_not_by_place(
    tmp_path: Path, ws: Workspace
):
    """A committed atlas is read from clones, so where it was built is not in it. What is in
    it is what a second machine can match a root by: kind, label, content hash, and the place
    in the checkout, which is the same in every clone."""
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    rows = atlas.generated["roots"]
    hashes = ws.root_hashes()

    assert [row["kind"] for row in rows] == ["install", "src"]
    assert [row["path"] for row in rows] == [
        "tests/fixtures/ws_launch/install",
        "tests/fixtures/ws_launch/src",
    ]
    assert all(row["hash"] == hashes[row["label"]] for row in rows)
    assert str(WS) not in json.dumps(atlas.generated)


def test_a_root_outside_the_repository_states_no_place_at_all(tmp_path: Path):
    """Nothing is invented for a root the checkout does not hold: it is named by its kind,
    its label and its content, and by nothing that is true of one machine only."""
    import shutil

    repo = tmp_path / "repo"
    shutil.copytree(WS / "src", repo / "src")
    (repo / ".git").mkdir(parents=True)
    shutil.copytree(WS / "install", tmp_path / "elsewhere" / "install")
    workspace = Workspace.discover(
        (
            RootSpec(kind="src", path=str(repo / "src")),
            RootSpec(kind="install", path=str(tmp_path / "elsewhere" / "install")),
        )
    )

    rows = {row["kind"]: row for row in workspace.root_records()}
    assert rows["src"]["path"] == "src"
    assert "path" not in rows["install"]
    assert str(tmp_path) not in json.dumps(rows)


def test_every_package_is_placed_relative_to_the_roots_that_hold_it(tmp_path: Path):
    """An editor opening a cited `file` id needs the package directory, and the root is the
    only frame that is true on every clone."""
    import shutil

    repo = tmp_path / "repo"
    shutil.copytree(WS / "src", repo / "src")
    shutil.copytree(WS / "install", tmp_path / "elsewhere" / "install")
    workspace = Workspace.discover(
        (
            RootSpec(kind="src", path=str(repo / "src")),
            RootSpec(kind="install", path=str(tmp_path / "elsewhere" / "install")),
        )
    )

    places = workspace.package_records()
    assert places
    assert all(places[name] for name in places)
    for rows in places.values():
        for label, inside in rows.items():
            assert label in {"src", "install"}
            assert not inside.startswith(("/", ".."))
    assert str(tmp_path) not in json.dumps(places)
    assert list(places) == sorted(places)


def test_a_root_with_no_checkout_around_it_is_given_no_place(tmp_path: Path):
    """`repo_root` falls back to the parent of the roots when nothing holds a `.git`, which
    is right for a `file:@repo` id and wrong here: a root at `/opt/ros/humble` with no
    repository anywhere was stated as sitting at `humble`, which is a directory name."""
    import shutil

    loose = tmp_path / "opt" / "humble"
    shutil.copytree(WS / "src", loose / "src")
    workspace = Workspace.discover((RootSpec(kind="src", path=str(loose / "src")),))

    assert workspace.checkout == ""
    assert workspace.in_repo(str(loose / "src")) == ""
    assert "path" not in workspace.root_records()[0]
    assert workspace.stated(["/opt/ros/humble/setup.bash"]) == ["/opt/ros/humble/setup.bash"]


def test_one_workspace_at_two_paths_is_one_file(tmp_path: Path):
    """The blocker this closes: every clone rewrote the committed atlas on its first hook
    run, so the work tree was dirty and `git merge` stopped before it started."""
    import shutil

    left = tmp_path / "a" / "ws"
    right = tmp_path / "b" / "a" / "much" / "longer" / "place" / "ws"
    written = []
    for target in (left, right):
        shutil.copytree(WS, target)
        workspace = Workspace.discover(
            (
                RootSpec(kind="src", path=str(target / "src")),
                RootSpec(kind="install", path=str(target / "install")),
            )
        )
        written.append(extract.run(_config(target / "out", workspace), workspace).to_bytes())

    assert written[0] == written[1]
    assert str(tmp_path).encode("utf-8") not in written[0]


def test_a_setup_file_in_the_checkout_is_stated_by_its_place_in_it(tmp_path: Path):
    """The same blocker one layer down, and the digest is the half that matters.

    A built workspace names `install/setup.bash`. The generation block states it as the
    checkout holds it and is not compared, but the input digest of every pass is compared,
    through the run id it builds. It held the absolute path, so one workspace at two paths
    was two atlases. Asserted on every run id and on the whole file, not on the block and the
    coded half: the launch collector held two more copies of the same absolute list, one in
    the profile attributes and one in its own digest, and the coded runs were the half that
    did not go through it.

    The generation block is inside the committed file even though it is outside the
    comparison, so it states a setup file the same way: by name and content for one outside
    every root, by its place in the checkout for one inside. Stating the machine path there
    cost a clone at another prefix a rewrite of the committed atlas on its first rebuild.
    """
    import shutil

    left = tmp_path / "a" / "ws"
    right = tmp_path / "b" / "a" / "much" / "longer" / "place" / "ws"
    stated = []
    runs = []
    files = []
    profiles = []
    for target in (left, right):
        shutil.copytree(WS, target)
        (target / ".git").mkdir()
        (target / "install" / "setup.bash").write_text("export FOO=1\n", encoding="utf-8")
        workspace = Workspace.discover(
            (
                RootSpec(kind="src", path=str(target / "src")),
                RootSpec(kind="install", path=str(target / "install")),
            )
        )
        cfg = Config(
            roots=tuple(RootSpec(kind=root.kind, path=root.path) for root in workspace.roots),
            profiles=PROFILES,
            out_dir=str(target / "out"),
            deterministic=True,
            ros_setup=("/opt/ros/humble/setup.bash", str(target / "install" / "setup.bash")),
        )
        atlas = extract.run(cfg, workspace)
        stated.append(atlas.generated["ros_setup"])
        runs.append(sorted(key for key, node in atlas.entities.items() if node.kind == "run"))
        files.append(atlas.to_bytes())
        profiles.append(
            {
                key: node.attrs["inputs"]["ros_setup"]
                for key, node in atlas.entities.items()
                if node.kind == "profile"
            }
        )

    assert stated[0] == stated[1]
    assert stated[0][0].startswith("@outside/setup.bash#")
    assert stated[0][1] == "install/setup.bash"
    assert runs[0] == runs[1]
    assert profiles[0] == profiles[1]
    for row in profiles[0].values():
        # A setup file inside the checkout is stated by its place in it. One outside every
        # root has no such place, so it is stated by identity, which is what two machines
        # with the distribution under different prefixes can still match on.
        outside, inside = row
        assert outside.startswith("@outside/setup.bash#")
        assert inside == "install/setup.bash"
    assert files[0] == files[1]
    assert str(tmp_path).encode("utf-8") not in files[0]


def test_one_distribution_at_two_prefixes_is_one_file(tmp_path: Path):
    """The other half of the same blocker: the ROS installation, not the checkout.

    Two boxes hold one distribution at two prefixes, and the setup file of each holds its own
    prefix. The generation block stated the path this box mounts it at, so a clone under a
    different prefix rewrote 29 bytes of the committed atlas before the developer had done
    anything and its work tree was dirty. Both prefixes here carry the same setup file with
    its own prefix inside it, which is what the build tool writes.
    """
    import shutil

    files = []
    stated = []
    for name in ("humble", "a/much/longer/place/humble"):
        prefix = tmp_path / "opt" / name
        prefix.mkdir(parents=True)
        setup = prefix / "setup.bash"
        setup.write_text(
            f"export AMENT_PREFIX_PATH={prefix}\nexport PATH={prefix}/bin:$PATH\n",
            encoding="utf-8",
        )
        target = tmp_path / "ws" / name
        shutil.copytree(WS, target)
        (target / ".git").mkdir()
        workspace = Workspace.discover(
            (
                RootSpec(kind="src", path=str(target / "src")),
                RootSpec(kind="install", path=str(target / "install")),
            )
        )
        cfg = Config(
            roots=tuple(RootSpec(kind=root.kind, path=root.path) for root in workspace.roots),
            profiles=PROFILES,
            out_dir=str(target / "out"),
            deterministic=True,
            ros_setup=(str(setup),),
        )
        atlas = extract.run(cfg, workspace)
        stated.append(atlas.generated["ros_setup"])
        files.append(atlas.to_bytes())

    assert stated[0] == stated[1]
    assert stated[0][0].startswith("@outside/setup.bash#")
    assert files[0] == files[1]
    assert str(tmp_path).encode("utf-8") not in files[0]


XACRO_LAUNCH = """\
import os

import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HERE = os.path.dirname(os.path.abspath(__file__))


def generate_launch_description():
    model = os.path.normpath(os.path.join(HERE, '..', 'urdf', 'bot.urdf.xacro'))
    text = xacro.process_file(model).toxml()
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=''),
        Node(package='robot_state_publisher', executable='robot_state_publisher', name='rsp',
             parameters=[{'robot_description': text,
                          'yaml_filename': LaunchConfiguration('map')}]),
    ])
"""


def _xacro_importable() -> bool:
    import importlib.util

    return importlib.util.find_spec("xacro") is not None


@pytest.mark.ros
@pytest.mark.skipif(not _xacro_importable(), reason="xacro is not importable")
def test_a_model_xacro_wrote_into_a_parameter_is_one_file_at_two_paths(tmp_path: Path):
    """xacro writes a banner naming the absolute path of its input into the text a node
    receives, a launch argument names a file by where this machine holds it, and a launch
    file that raises can quote its own path. Each put the mount point into compared data."""
    written = []
    for name in ("a/ws", "b/a/much/longer/place/ws"):
        target = tmp_path / name
        package = target / "src" / "bot"
        for folder in ("launch", "urdf", "maps"):
            (package / folder).mkdir(parents=True)
        (package / "package.xml").write_text(
            "<package format='3'><name>bot</name>"
            "<export><build_type>ament_python</build_type></export></package>",
            encoding="utf-8",
        )
        (package / "urdf" / "bot.urdf.xacro").write_text(
            '<robot name="bot" xmlns:xacro="http://www.ros.org/wiki/xacro">'
            '<link name="base"/></robot>',
            encoding="utf-8",
        )
        (package / "maps" / "map.yaml").write_text("image: map.pgm\n", encoding="utf-8")
        (package / "launch" / "bringup.launch.py").write_text(XACRO_LAUNCH, encoding="utf-8")
        (package / "launch" / "broken.launch.py").write_text(
            "import os\n\n\ndef generate_launch_description():\n"
            "    raise RuntimeError('cannot use ' + os.path.abspath(__file__))\n",
            encoding="utf-8",
        )
        workspace = Workspace.discover((RootSpec(kind="src", path=str(target / "src")),))
        profiles = (
            ProfileSpec(
                slug="bot",
                entrypoints=(
                    Entrypoint(
                        verb="launch",
                        pkg="bot",
                        target="bringup.launch.py",
                        args=(("map", str(package / "maps" / "map.yaml")),),
                    ),
                ),
            ),
            ProfileSpec(
                slug="broken",
                entrypoints=(Entrypoint(verb="launch", pkg="bot", target="broken.launch.py"),),
            ),
        )
        atlas = extract.run(_config(target / "out", workspace, profiles), workspace)
        written.append(atlas.canonical_bytes())
        description = atlas.entities["param:/rsp|robot_description"].candidates[0].value
        assert "@src/bot/urdf/bot.urdf.xacro" in description
        assert atlas.entities["param:/rsp|yaml_filename"].candidates[0].value == (
            "@src/bot/maps/map.yaml"
        )

    assert written[0] == written[1]
    assert str(tmp_path).encode("utf-8") not in written[0]
    assert b"cannot use bot/launch/broken.launch.py" in written[0]


def test_a_non_deterministic_run_stamps_the_time_outside_the_comparison(
    tmp_path: Path, ws: Workspace
):
    import dataclasses

    cfg = dataclasses.replace(_config(tmp_path / "out", ws), deterministic=False)
    atlas = extract.run(cfg, ws)
    assert atlas.generated["at"]
    stable = extract.run(_config(tmp_path / "other", ws), ws)
    assert atlas.canonical_bytes() == stable.canonical_bytes()


# ---------------------------------------------------------------------------
# selection and failure


def test_an_unknown_collector_is_a_usage_error(tmp_path: Path, ws: Workspace):
    with pytest.raises(UsageError):
        extract.run(_config(tmp_path / "out", ws), ws, collectors=["nope"])


def test_available_reports_the_call_order(tmp_path: Path, ws: Workspace):
    names = extract.available()
    assert names[: len(extract.ORDER)] == list(extract.ORDER)
    assert names.index("launch_declared") < names.index("params")


def test_every_committed_collector_comes_from_a_committed_module():
    """`MODULES` and `ORDER` are one decision written twice, so they have to agree. A
    collector whose module is not imported never registers and never runs, and a module
    imported for a collector nothing selects buys an import and no facts."""
    extract.available()
    assert {REGISTRY[name]().__class__.__module__ for name in extract.ORDER} == set(extract.MODULES)


def test_the_committed_order_names_both_halves_of_the_coded_layer():
    assert extract.default_collectors(CODED) == ["cpp_code", "py_code"]
    assert set(extract.default_collectors(LAYER)) == set(extract.STATIC) - {"cpp_code", "py_code"}


def test_a_runtime_collector_is_committed_and_never_in_a_default_selection():
    """A default extract reads the repository. Attaching to a live system is its own verb,
    and a recording is opened only when one was named, so neither can join by default."""
    assert extract.ORDER == extract.STATIC + extract.RUNTIME
    assert set(extract.RUNTIME) & set(extract.default_collectors()) == set()
    assert extract.collectors_for(OBSERVED) == ["proc", "probe"]
    assert extract.collectors_for(RECORDED) == ["bag", "stamps"]


def test_the_default_selection_is_the_committed_order_and_not_the_registry(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    """The registry fills up by import, so a default run that reads it depends on which
    module was imported first, which is a property of the caller and not of the workspace."""

    class Stray:
        name = "stray"
        layer = LAYER

        def collect(self, ctx: CollectContext) -> None:
            raise AssertionError("a collector nothing selected joined a default run")

    monkeypatch.setitem(REGISTRY, "stray", Stray)
    assert extract.default_collectors() == list(extract.STATIC)
    assert "stray" in extract.available(), "the registry still reports it"

    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    assert atlas.generated["collectors"] == list(extract.STATIC)


def test_a_layer_is_declared_present_only_when_a_collector_produced_a_fact(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    """A collector that declares its layer and then raises must not leave the layer
    claiming a truth. An empty layer and a failed one read the same otherwise."""

    class Exploding:
        name = "exploding"
        layer = CODED

        def collect(self, ctx: CollectContext) -> None:
            ctx.builder.declare_layer(CODED, present=True)
            raise RuntimeError("the source tree was not there")

    monkeypatch.setitem(REGISTRY, "exploding", Exploding)
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws, collectors=["exploding"])

    assert atlas.layers[CODED].present is False
    assert "produced a fact" in atlas.layers[CODED].reason
    assert atlas.layers[CODED].runs, "the run stays, so the failure is attributable"
    assert atlas.counters["layers_present"] == []
    assert [row["exc_type"] for row in extract.exceptions(atlas)] == ["RuntimeError"]


def test_a_collector_that_raises_is_recorded_and_not_fatal(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    class Exploding:
        name = "exploding"
        layer = LAYER

        def collect(self, ctx: CollectContext) -> None:
            raise RuntimeError("this collector reads a file that is not there")

    monkeypatch.setitem(REGISTRY, "exploding", Exploding)
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws, collectors=["exploding", "interfaces"])

    problems = extract.exceptions(atlas)
    assert [row["exc_type"] for row in problems] == ["RuntimeError"]
    assert problems[0]["path"] == "@collector/exploding"
    assert problems[0]["phase"] == "collect"
    assert "not there" in problems[0]["message"]
    assert schema.validate(atlas.to_json()) == []
    assert extract.out_path(cfg).is_file()


def test_a_message_names_a_path_outside_every_root_by_its_name_alone(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    """Every collector writes its own rows, and any of them can quote a path no root holds.
    The orchestrator states each row once, so none of them reaches compared data as a
    machine path, and a ROS name with a leading slash is left as it was."""
    elsewhere = f"{tmp_path}/somewhere/else/x.yaml"

    class Quoting:
        name = "quoting"
        layer = LAYER

        def collect(self, ctx: CollectContext) -> None:
            ctx.builder.add_run(
                LAYER,
                input_digest="00000000",
                exceptions=[
                    {
                        "path": "@collector/quoting",
                        "phase": "read",
                        "exc_type": "OSError",
                        "message": f"[Errno 2] No such file or directory: '{elsewhere}'",
                    }
                ],
            )
            raise RuntimeError(f"cannot remap /cmd_vel from {elsewhere}: it is not there")

    monkeypatch.setitem(REGISTRY, "quoting", Quoting)
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws, collectors=["quoting", "interfaces"])

    messages = sorted(row["message"] for row in extract.exceptions(atlas))
    assert messages == [
        "[Errno 2] No such file or directory: '@outside/x.yaml'",
        "cannot remap /cmd_vel from @outside/x.yaml: it is not there",
    ]
    assert str(tmp_path) not in atlas.canonical_bytes().decode()
    assert schema.validate(atlas.to_json()) == []


def test_a_schema_violation_is_raised_and_nothing_is_written(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    def one_violation(payload):
        return ["entities.x: invented"]

    monkeypatch.setattr(schema, "validate", one_violation)
    cfg = _config(tmp_path / "out", ws)
    with pytest.raises(ContractError):
        extract.run(cfg, ws)
    assert not extract.out_path(cfg).exists()


def test_the_run_records_the_inputs_that_decided_it(tmp_path: Path, ws: Workspace):
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws)
    runs = [node for node in atlas.by_kind("run") if node.attrs.get("roots")]
    assert runs
    for node in runs:
        assert set(node.attrs["roots"]) <= {"src", "install"}
        assert node.attrs["tool_version"]


def _digest_of(args_left, args_right) -> tuple[str, str]:
    left, right = (
        ProfileSpec(
            slug="two_entries",
            entrypoints=(
                Entrypoint(verb="launch", pkg="lx_bringup", target="a.launch.py", args=first),
                Entrypoint(verb="launch", pkg="lx_sensors", target="b.launch.py", args=second),
            ),
        )
        for first, second in (args_left, args_right)
    )
    step = extract.Pass(layer=LAYER, profile="two_entries", names=("launch",))
    cfg = Config(
        roots=(RootSpec(kind="src", path=str(WS / "src")),),
        profiles=(left,),
        out_dir="",
        deterministic=True,
    )
    workspace = Workspace.discover(cfg.roots)
    return (
        extract._input_digest(cfg, workspace, step, left, {}),
        extract._input_digest(cfg, workspace, step, right, {}),
    )


def test_an_argument_of_one_entry_point_is_not_an_argument_of_another():
    """AGENTS section 9.8: two configurations that really differ have to show an input
    difference. Flattened with no anchor, moving one argument from the first entry point to
    the second left the digest where it was, and `rosio diff` read two runs as one."""
    left, right = _digest_of(((("use_sim_time", "true"),), ()), ((), (("use_sim_time", "true"),)))
    assert left != right


def test_the_same_arguments_on_the_same_entry_points_are_one_digest():
    args = ((("map", "yard.yaml"), ("use_sim_time", "true")), (("params", "p.yaml"),))
    left, right = _digest_of(args, args)
    assert left == right


# ---------------------------------------------------------------------------
# citation through the package share


def _installed(root: Path, package: str, relative: str) -> Path:
    return root / "install" / package / "share" / package / relative


SHARED = "launch/sensors_only.launch.py"
"""Held by both trees of the fixture, which is what a package share read looks like."""


def test_a_file_read_through_the_share_says_so(tmp_path: Path, ws: Workspace):
    """The workspace, not the collector, is what knows the two trees hold one path."""
    ws.cite(str(_installed(WS, "lx_sensors", SHARED)))
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    entity = atlas.entities[ids.build_file("lx_sensors", SHARED)]
    assert entity.attrs["via_package_share"] is True
    assert "install_copy_differs" not in entity.attrs
    assert [finding for finding in atlas.findings if finding.rule == "build.stale"] == []


def test_a_stale_installed_copy_is_one_finding_on_the_file(tmp_path: Path):
    """One staleness finding, not a content difference on every value read from the file."""
    import shutil

    root = tmp_path / "bench"
    shutil.copytree(WS, root)
    installed = _installed(root, "lx_sensors", SHARED)
    installed.write_text("# drifted\n" + installed.read_text(encoding="utf-8"), encoding="utf-8")
    copy = Workspace.discover(
        (
            RootSpec(kind="src", path=str(root / "src")),
            RootSpec(kind="install", path=str(root / "install")),
        )
    )
    copy.cite(str(installed))
    atlas = extract.run(_config(tmp_path / "out", copy), copy)

    subject = ids.build_file("lx_sensors", SHARED)
    stale = [finding for finding in atlas.findings if finding.rule == "build.stale"]
    assert len(stale) == 1
    assert stale[0].subject == subject
    assert stale[0].severity == "warn"
    assert "sensors_only.launch.py" in stale[0].say
    assert ":~" not in stale[0].say
    assert atlas.entities[subject].attrs["install_copy_differs"] is True
    assert schema.validate(atlas.to_json()) == []


def test_the_walk_runs_the_installed_launch_file_and_names_the_stale_src(tmp_path: Path):
    """awk_ws: ros2 launch ran the installed bringup with /lidar/scan while the graph showed
    the src remap /front/scan, and no finding said the launch file had changed."""
    import shutil

    from rosio.collect import launch_declared

    root = tmp_path / "bench"
    shutil.copytree(WS, root)
    installed = _installed(root, "lx_sensors", SHARED)
    source = root / "src" / "lx_sensors" / SHARED
    source.write_text("# edited\n" + source.read_text(encoding="utf-8"), encoding="utf-8")
    copy = Workspace.discover(
        (
            RootSpec(kind="src", path=str(root / "src")),
            RootSpec(kind="install", path=str(root / "install")),
        )
    )
    index = launch_declared._launch_index(copy)
    assert index[("lx_sensors", SHARED)] == str(installed)
    copy.cite(index[("lx_sensors", SHARED)])
    atlas = extract.run(_config(tmp_path / "out", copy), copy)
    (stale,) = [item for item in atlas.findings if item.rule == "build.stale"]
    assert stale.subject == ids.build_file("lx_sensors", SHARED)


def test_a_share_read_of_a_file_no_collector_cited_is_ignored(tmp_path: Path, ws: Workspace):
    ws.cite(str(_installed(WS, "lx_bringup", "package.xml")))
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    marked = [node.id for node in atlas.entities.values() if node.attrs.get("via_package_share")]
    assert ids.build_file("lx_bringup", "package.xml") not in marked


# ---------------------------------------------------------------------------
# the drift engine, which is what makes the atlas a product


def test_the_rules_run_and_write_their_findings_onto_the_atlas(tmp_path: Path, ws: Workspace):
    from rosio.drift import rules as vocabulary

    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    reported = {finding.rule for finding in atlas.findings}

    assert atlas.findings, "an extraction that reports nothing is not the product"
    assert reported <= set(vocabulary.COMMITTED)
    assert atlas.counters["drift"]["rules_run"] == len(vocabulary.COMMITTED), (
        "every committed rule has to run, or a family that fails to load reads as a clean stack"
    )
    assert atlas.counters["drift"]["failed"] == []
    assert set(atlas.indexes["findings_by_rule"]) == reported
    assert "drift.findings" in atlas.capabilities


def test_the_findings_are_sorted_by_severity_and_counted(tmp_path: Path, ws: Workspace):
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    order = [model.SEVERITY_ORDER[finding.severity] for finding in atlas.findings]
    counted = dict.fromkeys(model.SEVERITY_ORDER, 0)
    for finding in atlas.findings:
        counted[finding.severity] += 1

    assert order == sorted(order), "the head of the array is the failure list"
    assert atlas.counters["findings"] == counted


def test_a_finding_no_rule_restates_survives_the_engine(tmp_path: Path, ws: Workspace):
    """The join writes node.ambiguous_identity and the rule that covers the runtime half of
    it skips a subject the join reported, so replacing the array would delete it."""
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    subject = sorted(node.id for node in atlas.by_kind("node"))[0]
    hand_written = model.Finding(
        rule="node.ambiguous_identity",
        severity="warn",
        subject=subject,
        title="two sources behind one executable state two node names",
        detail="written by the join, not by a rule",
        fix="name the node in the launch file",
        say="the join reported this one and no rule restates it",
    )
    atlas.findings.append(hand_written)

    extract.drift(atlas)

    assert hand_written.id in {finding.id for finding in atlas.findings}
    assert atlas.counters["drift"]["carried"] == 1


def test_running_the_engine_twice_over_one_atlas_changes_nothing(tmp_path: Path, ws: Workspace):
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    once = atlas.canonical_bytes()
    extract.drift(atlas)

    assert atlas.canonical_bytes() == once


def test_a_rule_that_raises_is_recorded_and_never_empties_the_report(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    from rosio.drift import rules as vocabulary

    def explode(atlas):
        raise RuntimeError("this rule reads a key that is not there")

    monkeypatch.setitem(vocabulary._REGISTRY, "param.shadowed", explode)
    atlas = extract.run(_config(tmp_path / "out", ws), ws)

    assert atlas.counters["drift"]["failed"] == ["param.shadowed"]
    assert atlas.findings, "one bad rule must not take the other thirty with it"
    assert schema.validate(atlas.to_json()) == []


def test_the_rule_families_are_discovered_and_not_listed():
    families = extract.rule_families()
    assert "params" in families and "topics" in families and "build" in families
    assert "rules" not in families and "engine" not in families
    assert families == sorted(families)


def test_what_the_discovery_loads_is_the_whole_committed_vocabulary():
    """A family that never imports reports nothing and reads exactly like a clean stack, so
    the count is asserted against the committed list and not against itself."""
    from rosio.drift import engine
    from rosio.drift import rules as vocabulary

    loaded, failures = engine.load_rules(extract.rule_families())

    assert failures == []
    assert {rule.id for rule in loaded} == set(vocabulary.COMMITTED)


# ---------------------------------------------------------------------------
# the runtime layers, and growing an atlas that already exists
#
# No ROS and no recorder are needed here. The probe collector takes the system it writes,
# which is the same split the launch collector uses, and a recording is three tables in a
# sqlite file.


def _observed_system():
    from rosio.collect import probe as probe_collect

    return probe_collect.ObservedSystem(
        nodes=[
            probe_collect.ObservedNode(
                name="talker",
                namespace="/",
                fqn="/talker",
                parameters=(probe_collect.Parameter(name="loop_period", value=20, type="integer"),),
            )
        ],
        endpoints=[
            probe_collect.Endpoint(
                kind="publisher",
                node="/talker",
                name="/chatter",
                type="std_msgs/msg/String",
                qos=probe_collect.Qos(history="keep_last", depth=10, reliability="reliable"),
            )
        ],
        rates=[probe_collect.TopicRate(topic="/chatter", count=42, window_s=5.0, hz=8.4)],
        domain_id="77",
        localhost_only="1",
    ).sort()


def _probe(system=None):
    from rosio.collect import probe as probe_collect

    return {PROBE: probe_collect.ProbeCollector(system=system or _observed_system())}


def _bag(directory: Path, topic: str = "/scan", count: int = 6) -> Path:
    """One small recording, written by hand so no recorder is needed."""
    import sqlite3

    import yaml

    directory.mkdir(parents=True)
    relative = f"{directory.name}_0.db3"
    connection = sqlite3.connect(str(directory / relative))
    connection.executescript(
        "CREATE TABLE schema(schema_version INTEGER PRIMARY KEY, ros_distro TEXT NOT NULL);"
        "CREATE TABLE topics(id INTEGER PRIMARY KEY, name TEXT NOT NULL, type TEXT NOT NULL,"
        " serialization_format TEXT NOT NULL, offered_qos_profiles TEXT NOT NULL);"
        "CREATE TABLE messages(id INTEGER PRIMARY KEY, topic_id INTEGER NOT NULL,"
        " timestamp INTEGER NOT NULL, data BLOB NOT NULL);"
    )
    connection.execute("INSERT INTO schema VALUES (3, 'humble')")
    connection.execute(
        "INSERT INTO topics VALUES (1, ?, 'sensor_msgs/msg/LaserScan', 'cdr', '')", (topic,)
    )
    for step in range(count):
        connection.execute(
            "INSERT INTO messages(topic_id, timestamp, data) VALUES (1, ?, ?)",
            (1750000000000000000 + step * 10000000, b"\x00" * 16),
        )
    connection.commit()
    connection.close()
    info = {
        "version": 5,
        "storage_identifier": "sqlite3",
        "relative_file_paths": [relative],
        "duration": {"nanoseconds": (count - 1) * 10000000},
        "starting_time": {"nanoseconds_since_epoch": 1750000000000000000},
        "message_count": count,
        "compression_format": "",
        "compression_mode": "",
        "ros_distro": "humble",
        "topics_with_message_count": [
            {
                "message_count": count,
                "topic_metadata": {
                    "name": topic,
                    "type": "sensor_msgs/msg/LaserScan",
                    "serialization_format": "cdr",
                    "offered_qos_profiles": "",
                },
            }
        ],
    }
    (directory / "metadata.yaml").write_text(
        yaml.safe_dump({"rosbag2_bagfile_information": info}, sort_keys=True), encoding="utf-8"
    )
    return directory


def test_the_observed_layer_is_added_to_an_atlas_that_already_exists(tmp_path: Path, ws: Workspace):
    """The point of the verb: a probe reaches a repository the static layers described, and
    re extracting to get there would throw that work away."""
    cfg = _config(tmp_path / "out", ws)
    before = extract.run(cfg, ws)
    declared = {node.id for node in before.entities.values() if LAYER in node.layers}

    after = extract.run(
        cfg, ws, collectors=[PROBE], opt_in=OPT_IN_PROBE, instances=_probe(), atlas=before
    )

    assert {node.id for node in after.entities.values() if LAYER in node.layers} == declared
    assert after.layers[OBSERVED].present
    assert after.layers[OBSERVED].root == model.Root.RUNTIME.value
    assert after.entities[ids.build_node("/talker")].fact(OBSERVED) is not None
    assert schema.validate(after.to_json()) == []
    assert extract.out_path(cfg).is_file()


@needs_cpp
def test_a_run_that_extends_an_atlas_recomputes_the_rule_findings(tmp_path: Path, ws: Workspace):
    """The rules see more layers than the run before them did, so the earlier rule output is
    computed again and never kept: a finding the new answer contradicts would otherwise stay
    in the report forever. What no rule produced is what survives."""
    cfg = _config(tmp_path / "out", ws)
    before = extract.run(cfg, ws)
    joined = model.Finding(
        rule="node.ambiguous_identity",
        severity="warn",
        subject=sorted(node.id for node in before.by_kind("node"))[0],
        title="two sources behind one executable state two node names",
        detail="written by the join, and no rule can derive it afterwards",
        fix="name the node in the launch file",
        say="the join reported this one",
    )
    before.findings.append(joined)
    extract.drift(before)
    stale = model.Finding(
        rule="param.shadowed",
        severity="warn",
        subject=sorted(node.id for node in before.by_kind("param"))[0],
        title="a finding the last run produced and this one does not",
        detail="no rule states this over the fixture workspace",
        fix="nothing",
        say="a value is set twice",
        discriminator="handmade",
    )
    before.findings.append(stale)

    after = extract.run(
        cfg, ws, collectors=[PROBE], opt_in=OPT_IN_PROBE, instances=_probe(), atlas=before
    )

    reported = {finding.id for finding in after.findings}
    assert joined.id in reported, "the join statement is not derivable and has to survive"
    assert stale.id not in reported, "the rule output is recomputed, not carried"


def test_the_capabilities_state_the_layers_the_atlas_holds(tmp_path: Path, ws: Workspace):
    """AGENTS section 9a: the capability is the feature probe, so it must describe the file
    and never what the tool can do."""
    cfg = _config(tmp_path / "out", ws)
    static = extract.run(cfg, ws)
    assert static.capabilities == sorted(extract.CAPABILITIES)
    assert "layer.observed" not in static.capabilities

    grown = extract.run(
        cfg, ws, collectors=[PROBE], opt_in=OPT_IN_PROBE, instances=_probe(), atlas=static
    )
    assert set(extract.LAYER_CAPABILITIES[OBSERVED]) <= set(grown.capabilities)
    assert grown.capabilities == sorted(set(grown.capabilities)), "sorted and unique"


def test_a_temp_path_folds_the_same_whatever_temp_root_this_process_has(monkeypatch):
    """The determinism the whole atlas rests on, at the one place it leaked.

    The launch child writes its rewritten parameters file under the temp root the launch code
    chose, which on the reference stack is a hardcoded `/tmp` while the orchestrator runs
    under another `TMPDIR`. Deciding from the asking process put a random name into a launch
    entity id, an entity label and an unresolved digest, so two runs of one input differed.
    """
    import tempfile

    from rosio.resolve import identity, records

    written = records.ParamSource(order=0, kind="external_file", path="/tmp/tmp9f2c_merged.yaml")
    other = records.ParamSource(order=0, kind="external_file", path="/tmp/tmpa41b_merged.yaml")
    elsewhere = records.ParamSource(
        order=0, kind="external_file", path="/mnt/scratch/x_merged.yaml"
    )
    for root in ("/tmp", "/mnt/scratch", "/var/tmp"):
        monkeypatch.setattr(tempfile, "tempdir", root)
        assert identity.under_temp("/tmp/tmp9f2c_merged.yaml"), root
        assert identity.param_token(written) == identity.param_token(other)
        assert identity.param_token(written) == identity.param_token(elsewhere)
        assert identity.EXTERNAL_TOKEN in identity.param_token(written)


def test_a_workspace_checked_out_under_tmp_keeps_its_own_files():
    """CI and pytest put whole workspaces under `/tmp`. A file inside a configured root is the
    workspace's own file there too, so its yaml stays provenance and keeps its name."""
    from rosio.resolve import identity, records

    root = "/tmp/pytest-of-ci/rover_ws"
    path = f"{root}/install/rover_bringup/share/rover_bringup/config/rover.yaml"
    assert identity.under_temp(path)
    assert not identity.under_temp(path, [root])
    assert identity.under_temp("/tmp/tmp9f2c_merged.yaml", [root])
    stated = records.ParamSource(order=0, kind="repo_file", path=path)
    assert identity.param_token(stated, [root]) == (
        "file:install/rover_bringup/share/rover_bringup/config/rover.yaml"
    )
    assert identity.relative_path(path, [root]).startswith("install/")


def test_the_capability_vocabulary_names_the_four_runtime_marks():
    """AGENTS section 9a names them, so the closed list a consumer tests against holds them
    whether or not this atlas reached the layer that carries them."""
    assert set(extract.ALL_CAPABILITIES) >= {
        "bagtopic.stats",
        "edge.timing",
        "layer.observed",
        "layer.recorded",
    }
    assert set(extract.CAPABILITIES) <= set(extract.ALL_CAPABILITIES)
    assert tuple(sorted(set(extract.ALL_CAPABILITIES))) == extract.ALL_CAPABILITIES


def test_every_committed_collector_is_registered_and_ordered():
    """The selection is the committed tuple and not the registry, so a runtime layer that is
    in one and not the other is a plain extract that silently drops it."""
    assert set(extract.ORDER) == set(extract.STATIC) | set(extract.RUNTIME)
    assert set(extract.ORDER) <= set(extract.available())
    for layer in (OBSERVED, RECORDED):
        assert extract.collectors_for(layer), f"{layer} has no committed collector"
        assert set(extract.collectors_for(layer)) & set(extract.RUNTIME)
    assert extract.default_collectors() == list(extract.STATIC)


def test_the_live_probe_is_not_reachable_without_an_explicit_opt_in(tmp_path: Path, ws: Workspace):
    """This box is attached to a vehicle. Selecting the probe by name has to be refused, and
    supplying a harmless instance must not buy the selection either."""
    cfg = _config(tmp_path / "out", ws)
    with pytest.raises(UsageError):
        extract.run(cfg, ws, collectors=[PROBE])
    with pytest.raises(UsageError):
        extract.run(cfg, ws, collectors=[PROBE], instances=_probe())
    assert PROBE not in extract.default_collectors()
    assert PROBE in extract.OPT_IN


def test_the_resolved_counters_carry_over_into_a_run_that_extends_an_atlas(
    tmp_path: Path, ws: Workspace
):
    """The denominator of the resolution rate belongs to the atlas, not to the last pass over
    it, or one probe turns a measured rate into a rate over four node names."""
    cfg = _config(tmp_path / "out", ws)
    before = extract.run(cfg, ws)
    counted = dict(before.counters["resolved"])

    after = extract.run(
        cfg, ws, collectors=[PROBE], opt_in=OPT_IN_PROBE, instances=_probe(), atlas=before
    )

    for want, count in counted.items():
        assert after.counters["resolved"][want] >= count
    assert after.counters["unresolved"] == before.counters["unresolved"]


def test_a_runtime_layer_keeps_the_reason_its_own_collector_stated(tmp_path: Path, ws: Workspace):
    """A probe that attached to a system with no node and a probe that never attached are two
    statements, and only the collector knows which one happened."""
    from rosio.collect import probe as probe_collect

    empty = probe_collect.ObservedSystem(reached=True)
    atlas = extract.run(
        _config(tmp_path / "out", ws),
        ws,
        collectors=[PROBE],
        opt_in=OPT_IN_PROBE,
        instances=_probe(empty),
    )
    status = atlas.layers[OBSERVED]

    assert status.present is False
    assert status.root == model.Root.RUNTIME.value
    assert "no node" in status.reason
    assert "produced a fact" not in status.reason, "the orchestrator overwrote the collector"


def test_a_runtime_collector_that_raises_leaves_a_reason_and_an_exception(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    class Exploding:
        name = PROBE
        layer = OBSERVED

        def collect(self, ctx: CollectContext) -> None:
            raise RuntimeError("rclpy is not importable here")

    atlas = extract.run(
        _config(tmp_path / "out", ws),
        ws,
        collectors=[PROBE],
        opt_in=OPT_IN_PROBE,
        instances={PROBE: Exploding()},
    )

    assert atlas.layers[OBSERVED].present is False
    assert atlas.layers[OBSERVED].reason
    assert [row["exc_type"] for row in extract.exceptions(atlas)] == ["RuntimeError"]


def test_two_runs_of_one_observed_system_are_byte_identical(tmp_path: Path, ws: Workspace):
    """The rate, the sample count and the window are volatile, so a second probe of a healthy
    system compares equal."""
    first = extract.run(
        _config(tmp_path / "a", ws),
        ws,
        collectors=[PROBE],
        opt_in=OPT_IN_PROBE,
        instances=_probe(),
        atlas=None,
    )
    second = extract.run(
        _config(tmp_path / "b", ws),
        ws,
        collectors=[PROBE],
        opt_in=OPT_IN_PROBE,
        instances=_probe(),
        atlas=None,
    )
    assert first.canonical_bytes() == second.canonical_bytes()


def test_the_recorded_layer_reads_the_recordings_it_was_given(tmp_path: Path, ws: Workspace):
    from rosio.collect.bag import BagCollector

    directory = _bag(tmp_path / "yard_small")
    atlas = extract.run(
        _config(tmp_path / "out", ws),
        ws,
        collectors=[BAG],
        instances={BAG: BagCollector(paths=(str(directory),))},
    )

    assert atlas.layers[RECORDED].present
    assert atlas.layers[RECORDED].root == model.Root.BAG.value
    assert [node.id for node in atlas.by_kind("bag")] == [ids.build_bag("yard_small")]
    assert [node.id for node in atlas.by_kind("bagtopic")] == [
        ids.build_bagtopic("yard_small", "/scan")
    ]
    assert set(extract.LAYER_CAPABILITIES[RECORDED]) <= set(atlas.capabilities)
    assert schema.validate(atlas.to_json()) == []


def test_a_recording_named_by_a_root_needs_no_further_request(tmp_path: Path, ws: Workspace):
    """`bag` is a root kind, so a config that names one describes a workspace with recordings
    in it and the collector finds them without a flag."""
    import dataclasses

    directory = _bag(tmp_path / "yard_rooted")
    cfg = dataclasses.replace(
        _config(tmp_path / "out", ws),
        roots=(*_config(tmp_path / "out", ws).roots, RootSpec(kind="bag", path=str(directory))),
    )
    atlas = extract.run(cfg, ws, collectors=[BAG])

    assert [node.id for node in atlas.by_kind("bag")] == [ids.build_bag("yard_rooted")]


def test_a_recording_states_no_profile(tmp_path: Path, ws: Workspace):
    """A recording does not know which composition produced it, so every fact holds under
    `*` and the profile columns stay honest."""
    from rosio.collect.bag import BagCollector

    directory = _bag(tmp_path / "yard_any")
    atlas = extract.run(
        _config(tmp_path / "out", ws),
        ws,
        collectors=[BAG],
        instances={BAG: BagCollector(paths=(str(directory),))},
    )
    facts = [
        fact for node in atlas.entities.values() for fact in node.facts if fact.layer == RECORDED
    ]
    assert facts
    assert {fact.profile for fact in facts} == {model.ANY_PROFILE}


def test_the_observation_is_filed_against_one_profile_only_when_one_was_named(
    tmp_path: Path, ws: Workspace
):
    """A probe cannot tell which composition it attached to. With one profile configured the
    caller said which, and with several the fact holds against each of them."""
    one = extract.run(
        _config(tmp_path / "one", ws, profiles=PROFILES[:1]),
        ws,
        collectors=[PROBE],
        opt_in=OPT_IN_PROBE,
        instances=_probe(),
    )
    many = extract.run(
        _config(tmp_path / "many", ws),
        ws,
        collectors=[PROBE],
        opt_in=OPT_IN_PROBE,
        instances=_probe(),
    )

    talker = ids.build_node("/talker")
    assert one.entities[talker].facts[0].profile == ids.build_profile("plain")
    assert many.entities[talker].facts[0].profile == model.ANY_PROFILE


# ---------------------------------------------------------------------------
# the summary


def test_summary_counts_what_the_atlas_holds(tmp_path: Path, ws: Workspace):
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    report = extract.summary(atlas)
    assert report["profiles"] == ["plain", "stack"]
    assert report["layers"] == sorted([CODED, LAYER]), "a default run reads both static layers"
    assert report["entities"] == len(atlas.entities)
    assert report["edges"] == len(atlas.edges)
    assert sum(report["entities_by_kind"].values()) == report["entities"]
    assert 0.0 <= report["resolution_rate"] <= 1.0
    assert report["unresolved"] == len(atlas.unresolved)


def test_summary_is_a_pure_function_of_the_atlas(tmp_path: Path, ws: Workspace):
    atlas = extract.run(_config(tmp_path / "out", ws), ws)
    assert extract.summary(atlas) == extract.summary(atlas)


# ---------------------------------------------------------------------------
# the size of an atlas, over an input a stranger holds

FIXTURE_CEILING_BYTES = 510_000
"""The size gate, over the committed fixture workspace. Measured in a clone at 467,916 bytes
with the launch walks executed against a sourced ROS 2, and at 300,600 without one, so the
number covers the larger of the two and carries 8.3 percent of headroom: adding a fixture
package is then a measurement and not a failure. A tree with no `.git` reads 100 bytes less
in both states, because `Workspace.checkout` then states no place for either root, and the
`generated` block is inside the file and outside `canonical_bytes`.

Both numbers come from `tests/fixtures`, which is in the clone, so anybody can reproduce,
check and raise them. AGENTS section 11 and section 1 rule 9 say why no gate beside them is
stated over the workspace this tool was developed against: nobody else has that tree, so
nobody else can reproduce, check or raise such a number, and the one that was published for
three waves went stale twice in them, both times because that workspace gained packages
rather than because the encoder changed."""

FIXTURE_BYTES_PER_ENTITY = 3_800
"""The same gate as a ratio, over the same committed input. Measured at 3,466 bytes per
entity with a sourced ROS 2 and 3,666 without one, and the number covers the larger of the
two with 3.5 percent of headroom. This is the gate that catches an encoder that started
writing more per row, because here the input cannot move under the number: a whole atlas
that grows because a fixture package arrived moves the ratio hardly at all."""


def test_the_fixture_atlas_stays_under_its_measured_ceiling(tmp_path: Path, ws: Workspace):
    """The whole size gate, over an input that does not move, so a red result is this
    repository and not the day.

    A third bound stood beside these two and read the ratio off the workspace this tool was
    developed against. It published two measurements of a tree nobody else holds, and it ran
    on no clone but that one, so what it asserted could be neither reproduced nor checked
    anywhere else. It is gone. The absence of `sets_param` it also carried is asserted below,
    and `tests/test_params.py` states the same absence against the parameter chain that used
    to write the edge. What went with it is the only run of the encoder at twenty megabytes,
    which is now measured privately on public stacks and not gated here.
    """
    atlas = extract.run(_config(tmp_path / "out", ws), ws)

    size = len(atlas.to_bytes())
    per_entity = size / len(atlas.entities)
    assert size <= FIXTURE_CEILING_BYTES, f"the fixture atlas grew to {size} bytes"
    assert per_entity <= FIXTURE_BYTES_PER_ENTITY, (
        f"the fixture atlas costs {per_entity:.0f} bytes per entity, "
        f"{size} bytes over {len(atlas.entities)} entities"
    )
    assert [edge for edge in atlas.edges.values() if edge.kind == "sets_param"] == []


# ---------------------------------------------------------------------------
# the command line


def test_cli_extract_writes_the_atlas_and_prints_the_counts(tmp_path: Path, capsys):
    from rosio.cli import main

    assert main(["extract", str(WS), "--out", str(tmp_path / "out")]) == 0
    printed = capsys.readouterr().out
    assert "profiles" in printed
    assert "wall time" in printed
    assert (tmp_path / "out" / "atlas.json").is_file()


def test_cli_json_summary_keeps_the_wall_time_volatile(tmp_path: Path, capsys):
    from rosio.cli import main

    assert main(["extract", str(WS), "--out", str(tmp_path / "out"), "--json"]) == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["volatile"]["wall_seconds"] >= 0.0
    assert payload["out"].endswith("atlas.json")
    assert payload["entities"] > 0
    assert set(payload) >= {"profiles", "entities_by_kind", "edges_by_kind", "resolution_rate"}


def test_cli_profile_selection_restricts_the_atlas(tmp_path: Path, capsys):
    from rosio.cli import main

    code = main(
        [
            "extract",
            str(WS),
            "--out",
            str(tmp_path / "out"),
            "--profile",
            "plain_node",
            "--json",
        ]
    )
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["profiles"] == ["plain_node"]


def test_cli_rejects_an_unknown_profile(tmp_path: Path, capsys):
    from rosio.cli import main
    from rosio.errors import EXIT_USAGE

    assert main(["extract", str(WS), "--out", str(tmp_path / "out"), "--profile", "nope"]) == (
        EXIT_USAGE
    )
    assert "unknown profile" in capsys.readouterr().err


def test_cli_bag_flag_adds_the_recorded_layer(tmp_path: Path, capsys):
    from rosio.cli import main

    directory = _bag(tmp_path / "yard_cli")
    code = main(
        ["extract", str(WS), "--out", str(tmp_path / "out"), "--bag", str(directory), "--json"]
    )
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert RECORDED in payload["layers"]
    assert payload["entities_by_kind"]["bagtopic"] == 1

    atlas = model.Atlas.loads((tmp_path / "out" / "atlas.json").read_text(encoding="utf-8"))
    assert [node.id for node in atlas.by_kind("bag")] == [ids.build_bag("yard_cli")]
    assert "layer.recorded" in atlas.capabilities


def test_cli_rejects_a_bag_that_is_not_there(tmp_path: Path, capsys):
    """Checked before anything runs, so a mistyped path never costs a whole extraction."""
    from rosio.cli import main
    from rosio.errors import EXIT_USAGE

    argv = ["extract", str(WS), "--out", str(tmp_path / "out"), "--bag", str(tmp_path / "nope")]
    assert main(argv) == EXIT_USAGE
    assert "bag directory does not exist" in capsys.readouterr().err
    assert not (tmp_path / "out").exists()


def test_cli_no_runtime_ignores_the_bags_the_config_names(tmp_path: Path, capsys):
    from rosio.cli import main

    directory = _bag(tmp_path / "yard_ignored")
    config = tmp_path / "rosio.yaml"
    config.write_text(
        "roots:\n"
        f"  - {{kind: src, path: {WS / 'src'}}}\n"
        f"  - {{kind: bag, path: {directory}}}\n"
        "profiles:\n"
        "  - slug: plain\n"
        "    entrypoints:\n"
        "      - launch: lx_bringup plain_node.launch.py\n",
        encoding="utf-8",
    )
    argv = ["extract", str(WS), "--config", str(config), "--out", str(tmp_path / "out"), "--json"]

    assert main(argv) == 0
    assert RECORDED in json.loads(capsys.readouterr().out)["layers"], "a bag root is enough"

    assert main([*argv, "--no-runtime"]) == 0
    assert RECORDED not in json.loads(capsys.readouterr().out)["layers"]


def test_cli_refuses_no_runtime_together_with_a_bag(tmp_path: Path, capsys):
    from rosio.cli import main
    from rosio.errors import EXIT_USAGE

    directory = _bag(tmp_path / "yard_both")
    argv = [
        "extract",
        str(WS),
        "--out",
        str(tmp_path / "out"),
        "--bag",
        str(directory),
        "--no-runtime",
    ]
    assert main(argv) == EXIT_USAGE
    assert "opposite things" in capsys.readouterr().err


@needs_cpp
def test_cli_layer_flag_reads_that_layer_and_only_that_layer(tmp_path: Path, capsys):
    """Both halves of the coded layer run, and the declared collectors stay out of it."""
    from rosio.cli import main

    code = main(
        ["extract", str(CODE_WS), "--out", str(tmp_path / "out"), "--layer", "coded", "--json"]
    )
    assert code == 0
    payload = json.loads(capsys.readouterr().out)
    assert payload["layers"] == [CODED]
    assert payload["entities_by_kind"].get("launchfile", 0) == 0, "no declared collector ran"

    atlas = model.Atlas.loads((tmp_path / "out" / "atlas.json").read_text(encoding="utf-8"))
    packages = {ids.parse_sym(node.id).package for node in atlas.by_kind("sym")}
    assert {"cx_nodes", "px_nodes"} <= packages, "the C++ and the python halves both ran"
    assert all(fact.layer == CODED for node in atlas.entities.values() for fact in node.facts)


def test_cli_rejects_a_workspace_that_is_not_there(tmp_path: Path, capsys):
    from rosio.cli import main
    from rosio.errors import EXIT_USAGE

    assert main(["extract", str(tmp_path / "nowhere")]) == EXIT_USAGE
    assert "does not exist" in capsys.readouterr().err


def test_cli_fail_on_violation_reports_a_recorded_problem(tmp_path: Path, capsys):
    """The ROS 1 launch file in the fixture is refused, recorded, and never fatal."""
    from rosio.cli import main
    from rosio.errors import EXIT_CONTRACT

    # The fallback leaves a ROS 1 file out, so the profile that names it is written down.
    config = tmp_path / "rosio.yaml"
    config.write_text(
        "profiles:\n"
        "  - slug: legacy_ros1\n"
        "    entrypoints:\n"
        "      - launch: lx_bringup legacy_ros1.launch\n",
        encoding="utf-8",
    )
    argv = ["extract", str(WS), "--config", str(config), "--out", str(tmp_path / "out")]
    assert main(argv) == 0
    assert main([*argv, "--fail-on-violation"]) == EXIT_CONTRACT
    assert "1 problem recorded during extraction" in capsys.readouterr().err


def test_cli_out_flag_wins_over_the_config(tmp_path: Path, ws: Workspace):
    from rosio.cli import main

    config = tmp_path / "rosio.yaml"
    config.write_text(
        "out: ignored-by-the-flag\n"
        "profiles:\n"
        "  - slug: plain\n"
        "    entrypoints:\n"
        "      - launch: lx_bringup plain_node.launch.py\n",
        encoding="utf-8",
    )
    code = main(
        [
            "extract",
            str(WS),
            "--config",
            str(config),
            "--out",
            str(tmp_path / "out"),
            "--quiet",
        ]
    )
    assert code == 0
    assert (tmp_path / "out" / "atlas.json").is_file()
    assert not (tmp_path / "ignored-by-the-flag").exists()


# ---------------------------------------------------------------------------
# a pass that did not finish, which the resolution rate cannot see


class Dying:
    """A collector whose pass stops with data still to read."""

    name = "cpp_code"
    layer = CODED

    def collect(self, ctx: CollectContext) -> None:
        raise RuntimeError("the source tree went away halfway through")


class Empty:
    """A collector with nothing to read, which is complete and not failed."""

    name = "urdf"
    layer = LAYER

    def inputs_present(self, ctx: CollectContext) -> bool:
        return False

    def collect(self, ctx: CollectContext) -> None:
        raise AssertionError("a collector with no inputs must not be attempted")


SOUND = ("interfaces", "plugins")
"""A selection whose collectors read the fixture workspace and finish, so the loss under
test is the constructed one and not an optional dependency this box does not hold."""


def test_a_dead_pass_is_counted_named_and_graded(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    """The whole point: an atlas that dropped a pass used to carry zero findings, a
    resolution rate of one and a passing error gate. The counter and the rule are what the
    gate reads, because a ratio loses its own denominator when a pass dies."""
    extract.available()  # the registry fills by import, and a stand in must replace the real one
    monkeypatch.setitem(REGISTRY, "cpp_code", Dying)
    cfg = _config(tmp_path / "out", ws)
    atlas = extract.run(cfg, ws, collectors=[*SOUND, "cpp_code"])

    passes = atlas.counters["passes"]
    assert passes["complete"] is False
    assert passes["refused"] == 1 and passes["refused_collectors"] == ["cpp_code"]
    assert passes["attempted"] == passes["completed"] + passes["refused"]

    found = [item for item in atlas.findings if item.rule == "extract.pass_incomplete"]
    assert len(found) == 1
    assert found[0].severity == "error"
    assert "cpp_code" in found[0].say and "went away halfway through" in found[0].say
    assert extract.contribution("cpp_code") in found[0].say
    assert schema.validate(atlas.to_json()) == []


def test_the_gate_fails_on_a_dead_pass_and_passes_without_one(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    from rosio.cli import main
    from rosio.errors import EXIT_DRIFT

    out = tmp_path / "out"
    whole = extract.run(_config(out, ws), ws, collectors=list(SOUND))
    assert whole.counters["passes"]["complete"] is True
    assert main(["drift", "--atlas", str(out / "atlas.json"), "--fail-on", "error"]) == 0

    extract.available()  # the registry fills by import, and a stand in must replace the real one
    monkeypatch.setitem(REGISTRY, "cpp_code", Dying)
    dead = tmp_path / "dead"
    extract.run(_config(dead, ws), ws, collectors=[*SOUND, "cpp_code"])
    code = main(["drift", "--atlas", str(dead / "atlas.json"), "--fail-on", "error"])
    assert code == EXIT_DRIFT


def test_a_collector_with_nothing_to_read_does_not_fire_the_rule(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    """Otherwise the rule cries wolf on every small repository and is ignored within a
    week. An absent input is a state of its own and never a loss."""
    extract.available()
    monkeypatch.setitem(REGISTRY, "urdf", Empty)
    atlas = extract.run(_config(tmp_path / "out", ws), ws, collectors=[*SOUND, "urdf"])

    passes = atlas.counters["passes"]
    assert passes["complete"] is True and passes["not_attempted"] == 1
    assert [item for item in atlas.findings if item.rule == "extract.pass_incomplete"] == []
    rows = [row for _, row in model.passes_of(atlas) if row["collector"] == "urdf"]
    assert rows and all(row["state"] == "not_attempted" for row in rows)


def test_the_report_says_the_atlas_is_not_whole_at_the_top(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    from rosio.render.report import render_report

    whole = render_report(extract.run(_config(tmp_path / "out", ws), ws, collectors=list(SOUND)))
    assert "collector passes finished" in whole
    assert "not whole" not in whole

    extract.available()  # the registry fills by import, and a stand in must replace the real one
    monkeypatch.setitem(REGISTRY, "cpp_code", Dying)
    atlas = extract.run(_config(tmp_path / "dead", ws), ws, collectors=[*SOUND, "cpp_code"])
    head = render_report(atlas).split("## Node inventory")[0]
    assert "This atlas is not whole" in head
    assert "cpp_code" in head and "lower bound" in head


# ---------------------------------------------------------------------------
# hostile inputs, the standing detector
#
# One class of defect keeps coming back and is invisible by construction: an input the
# extraction cannot follow costs entities, edges and a whole want, while the finding set
# stays the control's, every remaining rate stays at 1.000 and the error gate exits zero.
# The atlas reads healthier the more it lost. The fixtures below are the shapes it arrived
# in, and the detector is the question asked of each of them: did anything go missing with
# nothing a reader meets.

HOSTILE_PACKAGE = """<?xml version="1.0"?>
<package format="3">
  <name>demo_pkg</name>
  <version>0.1.0</version>
  <description>the hostile input bench</description>
  <maintainer email="bench@example.invalid">bench</maintainer>
  <license>Apache-2.0</license>
</package>
"""

MSG_SOUND = "float64 LIMIT=1.5\nfloat64 rate\n"
MSG_UNREPRESENTABLE = "float64 LIMIT=inf\nfloat64 rate\n"

PY_SOUND = """from rclpy.node import Node


class Worker(Node):
    def __init__(self):
        super().__init__('worker')
        self.pub = self.create_publisher(String, '/hz/out', 10)
"""
PY_UNPARSABLE = "def worker(:\n    pass\n"

HOSTILE_COLLECTORS = ("interfaces", "entrypoints", "py_code")
"""Enough layers to lose something in each, and no optional dependency, so a failure here
is the constructed one and never this box."""


def _hostile_ws(root: Path, *, msg: str = MSG_SOUND, source: str = PY_SOUND) -> Path:
    package = root / "src" / "demo_pkg"
    (package / "msg").mkdir(parents=True)
    (package / "demo_pkg").mkdir(parents=True)
    (package / "package.xml").write_text(HOSTILE_PACKAGE, encoding="utf-8")
    (package / "msg" / "Thing.msg").write_text(msg, encoding="utf-8")
    (package / "demo_pkg" / "worker.py").write_text(source, encoding="utf-8")
    (package / "run.sh").write_text("ros2 launch demo_pkg demo.launch.py\n", encoding="utf-8")
    return root


def _hostile_atlas(root: Path, collectors=HOSTILE_COLLECTORS, bag: Path | None = None):
    roots = [RootSpec(kind="src", path=str(root / "src"))]
    if bag is not None:
        roots.append(RootSpec(kind="bag", path=str(bag)))
    workspace = Workspace.discover(tuple(roots))
    cfg = Config(
        roots=tuple(roots),
        profiles=(),
        out_dir=str(root / "rosio-out"),
        deterministic=True,
    )
    return extract.run(cfg, workspace, collectors=list(collectors))


def _lost(control: model.Atlas, atlas: model.Atlas) -> tuple[list[str], list[str]]:
    """The entity kinds and the resolution wants the control holds and this atlas does not.

    A want that leaves the counter is the sharper half: the rate is resolved over resolved
    plus unresolved per want, so a want with no site left reads as a perfect score over a
    denominator nobody can see shrinking.
    """
    kinds = dict(control.counters.get("entities_by_kind") or {})
    mine = dict(atlas.counters.get("entities_by_kind") or {})
    gone_kinds = sorted(kind for kind, count in kinds.items() if mine.get(kind, 0) < count)
    wants = set(control.counters.get("resolution_rate") or {})
    gone_wants = sorted(wants - set(atlas.counters.get("resolution_rate") or {}))
    return gone_kinds, gone_wants


def _met(control: model.Atlas, atlas: model.Atlas) -> list[str]:
    """Everything a reader of this atlas meets that the control did not put there."""
    known = {item.fingerprint for item in control.findings}
    met = [f"finding {item.rule}" for item in atlas.findings if item.fingerprint not in known]
    met += [f"unresolved {entry.want} on {entry.owner}" for entry in atlas.unresolved]
    met += [
        f"pass {row.get('collector')} {row.get('state')}"
        for _run, row in model.passes_of(atlas)
        if row.get("state") == model.PassState.REFUSED.value
    ]
    return sorted(met)


def _value_that_cannot_be_represented(tmp_path: Path):
    control = _hostile_atlas(_hostile_ws(tmp_path / "sound"))
    hostile = _hostile_atlas(_hostile_ws(tmp_path / "hostile", msg=MSG_UNREPRESENTABLE))
    return control, hostile, "type"


def _file_that_cannot_be_parsed(tmp_path: Path):
    control = _hostile_atlas(_hostile_ws(tmp_path / "sound"))
    hostile = _hostile_atlas(_hostile_ws(tmp_path / "hostile", source=PY_UNPARSABLE))
    return control, hostile, "file"


def _recording_that_is_damaged(tmp_path: Path):
    sound = _hostile_ws(tmp_path / "sound")
    _bag(sound / "bags" / "yard")
    control = _hostile_atlas(sound, collectors=[BAG], bag=sound / "bags")

    broken = _hostile_ws(tmp_path / "hostile")
    directory = _bag(broken / "bags" / "yard")
    (directory / f"{directory.name}_0.db3").write_bytes(b"")
    hostile = _hostile_atlas(broken, collectors=[BAG], bag=broken / "bags")
    return control, hostile, "bag"


HOSTILE = {
    "a value that cannot be represented": _value_that_cannot_be_represented,
    "a file that cannot be parsed": _file_that_cannot_be_parsed,
    "a recording that is damaged": _recording_that_is_damaged,
}


@pytest.mark.parametrize("shape", sorted(HOSTILE), ids=lambda name: name.replace(" ", "_"))
def test_a_hostile_input_never_costs_data_with_nothing_a_reader_meets(shape: str, tmp_path: Path):
    """The detector, as a standing test. Each shape lost something here once, with the
    control's finding set, the control's rates and a passing gate, which is the one failure
    mode a reader of the atlas cannot see."""
    control, hostile, owner_kind = HOSTILE[shape](tmp_path)

    assert _met(control, control) == [], "the control is the healthy half and states nothing"
    met = _met(control, hostile)
    assert met, f"{shape} lost data and the atlas says nothing about it"
    assert any(item.startswith("unresolved ") and owner_kind in item for item in met), (
        f"{shape} has to leave its blind spot on the {owner_kind} that owns the site"
    )
    assert schema.validate(hostile.to_json()) == []


@pytest.mark.parametrize("shape", sorted(HOSTILE), ids=lambda name: name.replace(" ", "_"))
def test_what_a_hostile_input_took_out_of_the_counter_is_visible(shape: str, tmp_path: Path):
    """A want that leaves the counter is the sharpest half of this class.

    The rate is resolved over resolved plus unresolved per want, so a want with no site
    left scores nothing at all and every remaining rate reads perfect over a smaller
    denominator. AGENTS section 13 asks the collector to leave the denominator behind. The
    least this has to hold is that the entity kinds and the wants that went are covered by
    a blind spot a reader meets, and never by an exception row nothing grades.
    """
    control, hostile, _owner_kind = HOSTILE[shape](tmp_path)
    gone_kinds, gone_wants = _lost(control, hostile)
    blind = [item for item in _met(control, hostile) if item.startswith("unresolved ")]

    if gone_kinds or gone_wants:
        assert blind, (
            f"{shape} dropped {', '.join([*gone_kinds, *gone_wants])} with no blind spot to meet"
        )
    for want, rate in (hostile.counters.get("resolution_rate") or {}).items():
        if want in gone_wants:
            continue
        assert rate <= (control.counters.get("resolution_rate") or {}).get(want, 1.0), (
            f"{shape} improved the {want} rate while it was losing data"
        )


def test_a_root_with_no_file_is_not_a_loss_and_the_gate_stays_green(tmp_path: Path, capsys):
    """The opposite failure. A configured root holding no file is a workspace where nothing
    is wrong, and a rule that fires on it is switched off within a week, which reopens the
    hole it closes behind a false sense of safety."""
    from rosio.cli import main

    (tmp_path / "src").mkdir()
    atlas = _hostile_atlas(tmp_path)

    passes = atlas.counters["passes"]
    assert passes["complete"] is True and passes["refused_collectors"] == []
    rows = {row["collector"]: row for _run, row in model.passes_of(atlas)}
    assert rows["entrypoints"]["state"] == model.PassState.NOT_ATTEMPTED.value
    assert rows["entrypoints"]["reason"]
    assert [item for item in atlas.findings if item.rule == "extract.pass_incomplete"] == []

    written = tmp_path / "rosio-out" / "atlas.json"
    assert main(["drift", "--atlas", str(written), "--fail-on", "error"]) == 0
    capsys.readouterr()


def test_a_run_that_stops_early_stops_the_walks_it_started(
    tmp_path: Path, ws: Workspace, monkeypatch: pytest.MonkeyPatch
):
    closed: list[bool] = []
    monkeypatch.setattr(CollectContext, "close", lambda self: closed.append(True), raising=False)

    def interrupted(*_args, **_kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(extract, "_run_pass", interrupted)
    with pytest.raises(KeyboardInterrupt):
        extract.run(_config(tmp_path, ws), ws)
    assert closed == [True]
