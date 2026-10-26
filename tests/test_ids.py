"""Id grammar: round trips, escaping, refusals, ROS name normalisation."""

import pytest

from rosio import ids
from rosio.errors import ContractError

ROLE = ids.role_digest(argv=["--frame-id", "base_footprint"], remaps=[])


def test_every_kind_round_trips():
    cases = [
        (ids.build_pkg("nx_bringup"), ids.parse_pkg, "nx_bringup"),
        (
            ids.build_file("nx_perception", "src/detector_node.cpp"),
            ids.parse_file,
            ids.FileId("nx_perception", "src/detector_node.cpp"),
        ),
        (
            ids.build_file(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py"),
            ids.parse_file,
            ids.FileId(ids.REPO_PKG, "dx_app/dx_launcher/profiles.py"),
        ),
        (
            ids.build_sym(
                "nx_bridge_cpp",
                "src/io_bridge_lifecycle.cpp",
                "nx_bridge_cpp::IoBridgeLifecycle::on_configure",
            ),
            ids.parse_sym,
            ids.SymId(
                "nx_bridge_cpp",
                "src/io_bridge_lifecycle.cpp",
                "nx_bridge_cpp::IoBridgeLifecycle::on_configure",
            ),
        ),
        (
            ids.build_exe("nx_bridge", "pose_health"),
            ids.parse_exe,
            ids.ExeId("nx_bridge", "pose_health"),
        ),
        (
            ids.build_launchfile("nx_bringup", "launch/sensors.launch.py"),
            ids.parse_launchfile,
            ids.FileId("nx_bringup", "launch/sensors.launch.py"),
        ),
        (ids.build_profile("nx_navigation"), ids.parse_profile, "nx_navigation"),
        (ids.build_node("/ouster/os_driver"), ids.parse_node, ids.NodeId(fqn="/ouster/os_driver")),
        (ids.build_topic("/ouster/points"), ids.parse_topic, "/ouster/points"),
        (
            ids.build_service("/navigate_to_pose/_action/cancel_goal"),
            ids.parse_service,
            "/navigate_to_pose/_action/cancel_goal",
        ),
        (ids.build_action("/navigate_to_pose"), ids.parse_action, "/navigate_to_pose"),
        (ids.build_frame("/os_sensor"), ids.parse_frame, "os_sensor"),
        (
            ids.build_type("nx_msgs", "msg", "RobotStatus"),
            ids.parse_type,
            ids.TypeId("nx_msgs", "msg", "RobotStatus"),
        ),
        (
            ids.build_plugin("nav2_core::Controller", "nx_controller/DelayAwareController"),
            ids.parse_plugin,
            ids.PluginId("nav2_core::Controller", "nx_controller/DelayAwareController"),
        ),
        (
            ids.build_pparam("nav2_core::Controller", "nx_controller/X", "state_timeout"),
            ids.parse_pparam,
            ids.PluginParamId("nav2_core::Controller", "nx_controller/X", "state_timeout"),
        ),
        (
            ids.build_run("declared", "nx_navigation", "9f2c1a4b"),
            ids.parse_run,
            ids.RunId("declared", "nx_navigation", "9f2c1a4b"),
        ),
        (ids.build_bag("2026_08_12-14_03_05_site"), ids.parse_bag, "2026_08_12-14_03_05_site"),
        (
            ids.build_bagtopic("2026_08_12-14_03_05_site", "/ouster/points"),
            ids.parse_bagtopic,
            ids.BagTopicId("2026_08_12-14_03_05_site", "/ouster/points"),
        ),
    ]
    for built, parser, want in cases:
        assert parser(built) == want, built


def test_launch_entity_round_trip():
    steps = (
        ids.LaunchStep("OpaqueFunction", "setup"),
        ids.LaunchStep("OnProcessExit", "ready_gate"),
        ids.LaunchStep("LifecycleNode", "os_driver"),
    )
    built = ids.build_launchent("nx_bringup", "launch/sensors.launch.py", steps)
    parsed = ids.parse_launchent(built)
    assert parsed == ids.LaunchEntId("nx_bringup", "launch/sensors.launch.py", steps)
    assert ids.kind_of(built) == "launchent"


def test_synthetic_node_and_derived_ids_round_trip():
    node_id = ids.build_node_synthetic("nav2_controller", "controller_server", "/", ROLE)
    assert ids.is_synthetic(node_id)
    assert ids.parse_node(node_id) == ids.NodeId(
        fqn=None,
        package="nav2_controller",
        executable="controller_server",
        namespace="/",
        role=ROLE,
        synthetic=True,
    )
    param_id = ids.build_param(node_id, "FollowPath.plugin")
    assert ids.is_synthetic(param_id)
    assert ids.parse_param(param_id) == ids.ParamId(owner=node_id, name="FollowPath.plugin")
    pinst_id = ids.build_pinst(node_id, "FollowPath")
    assert ids.parse_pinst(pinst_id) == ids.PluginInstanceId(owner=node_id, instance="FollowPath")


def test_named_param_and_pinst_round_trip():
    owner = ids.build_node("/local_costmap/local_costmap")
    param_id = ids.build_param(owner, "width")
    assert param_id == "param:/local_costmap/local_costmap|width"
    assert ids.parse_param(param_id) == ids.ParamId(owner=owner, name="width")
    pinst_id = ids.build_pinst(ids.build_node("/controller_server"), "FollowPath")
    assert pinst_id == "pinst:/controller_server|FollowPath"


def test_edge_and_finding_ids_round_trip_with_compound_endpoints():
    src = ids.build_file("nx_bringup", "config/nav2_params.yaml")
    dst = ids.build_param(ids.build_node("/controller_server"), "FollowPath.plugin")
    handle = ids.build_edge("sets_param", src, dst, key="a1b2c3d4")
    assert ids.parse_edge(handle) == ids.EdgeId("sets_param", src, dst, "a1b2c3d4")
    finding = ids.build_finding("param.shadowed", dst, "nav2_params")
    assert ids.parse_finding(finding) == ids.FindingId("param.shadowed", dst, "nav2_params")


def test_edge_with_no_key_round_trips_to_an_empty_key():
    handle = ids.build_edge("has_param", ids.build_node("/x"), "param:/x|y")
    assert ids.parse_edge(handle).key == ""


def test_separator_inside_a_name_is_escaped_and_recovered():
    weird = "topic|with%bar and space"
    built = ids.build("pkg", weird)
    assert "|" not in built.split(":", 1)[1]
    assert ids.parse(built).segments == (weird,)
    assert ids.escape_segment("a|b%c d") == "a%7Cb%25c%20d"
    assert ids.unescape_segment("a%7Cb%25c%20d") == "a|b%c d"


def test_non_ascii_segment_survives_a_round_trip():
    built = ids.build("pkg", "grün")
    assert ids.parse(built).segments == ("grün",)


def test_a_leading_tilde_in_a_name_cannot_fake_a_synthetic_body():
    # a bag directory named ~backup and a frame named ~odom are legal inputs, and
    # neither may claim the synthetic marker.
    for built, parser, want in (
        (ids.build_bag("~backup"), ids.parse_bag, "~backup"),
        (ids.build_frame("~odom"), ids.parse_frame, "~odom"),
        (ids.build_pkg("~x"), ids.parse_pkg, "~x"),
    ):
        assert not ids.parse(built).synthetic, built
        assert parser(built) == want
    assert ids.escape_segment("~odom") == "%7Eodom"
    assert ids.build_bag("~backup") == "bag:%7Ebackup"


def test_unknown_kind_parses_but_is_not_an_entity():
    parsed = ids.parse("futurekind:something")
    assert parsed.kind == "futurekind"
    assert not ids.is_entity("futurekind:something")


@pytest.mark.parametrize(
    "bad",
    [
        "",
        "nocolon",
        "Topic:/x",
        "topic:",
        "1kind:x",
    ],
)
def test_malformed_ids_are_refused(bad):
    with pytest.raises(ContractError):
        ids.parse(bad)


def test_segment_count_is_checked():
    with pytest.raises(ContractError):
        ids.parse("file:only_one_segment")
    with pytest.raises(ContractError):
        ids.parse("topic:/a|/b")


def test_ordinal_discriminators_are_refused():
    with pytest.raises(ContractError):
        ids.build_launchent(
            "nx_bringup", "launch/x.launch.py", (ids.LaunchStep("OpaqueFunction", "0"),)
        )
    with pytest.raises(ContractError):
        ids.build_finding("param.shadowed", "param:/x|y", "3")


def test_file_ids_refuse_machine_paths():
    with pytest.raises(ContractError):
        ids.build_file("nx_perception", "/srv/nx_ws/src/x.cpp")
    with pytest.raises(ContractError):
        ids.build_file("nx_perception", "../outside.cpp")
    assert ids.parse_file(ids.build_file("p", "./src//a.cpp")).relpath == "src/a.cpp"


def test_synthetic_form_only_exists_where_it_is_defined():
    with pytest.raises(ContractError):
        ids.build("topic", "/x", synthetic=True)


def test_label_never_leaks_a_synthetic_body():
    node_id = ids.build_node_synthetic("nav2_controller", "controller_server", "/", ROLE)
    assert ids.label_for(node_id) == "nav2_controller/controller_server@/"
    assert ids.label_for(ids.build_topic("/ouster/points")) == "/ouster/points"


@pytest.mark.parametrize(
    ("name", "namespace", "node_name", "want"),
    [
        ("/ouster/points", "/", None, "/ouster/points"),
        ("input", "/ouster", None, "/ouster/input"),
        ("cmd_vel", "/", None, "/cmd_vel"),
        ("ouster/os_driver", "/", None, "/ouster/os_driver"),
        ("~/params", "/ns", "supervisor", "/ns/supervisor/params"),
        ("~params", "/", "supervisor", "/supervisor/params"),
        ("~", "/ns", "supervisor", "/ns/supervisor"),
        ("//double//slash/", "/", None, "/double/slash"),
        ("/navigate_to_pose/_action/status", "/", None, "/navigate_to_pose/_action/status"),
    ],
)
def test_ros_name_normalisation(name, namespace, node_name, want):
    assert ids.normalise_ros_name(name, namespace, node_name) == want


def test_relative_parameter_values_use_the_same_engine_as_remaps():
    # A launch file gives lifecycle_manager node_names ['ouster/os_driver'] at
    # namespace /, so the value must land on the same id as the node itself.
    assert ids.build_node(ids.normalise_ros_name("ouster/os_driver", "/")) == ids.build_node(
        "/ouster/os_driver"
    )


def test_ros_name_failures():
    with pytest.raises(ContractError):
        ids.normalise_ros_name("")
    with pytest.raises(ContractError):
        ids.normalise_ros_name("~private", "/", None)
    with pytest.raises(ContractError):
        ids.normalise_ros_name("/has-a-dash")
    assert not ids.is_valid_ros_name("/has space")
    assert ids.is_valid_ros_name("relative_ok")


def test_namespace_and_relative_helpers():
    assert ids.normalise_namespace("") == "/"
    assert ids.normalise_namespace("ouster/") == "/ouster"
    assert ids.normalise_namespace(None) == "/"
    assert ids.join_namespace("/", "x") == "/x"
    assert ids.join_namespace("/a", "/b") == "/a/b"
    assert ids.is_relative("input")
    assert not ids.is_relative("/input")
    assert not ids.is_relative("~input")
    assert ids.is_private("~input")
    assert ids.split_ros_name("/local_costmap/local_costmap") == (
        "/local_costmap",
        "local_costmap",
    )


def test_parameter_names_normalise_to_one_dotted_form():
    assert ids.normalise_param_name("ground.max_slope") == "ground.max_slope"
    assert ids.normalise_param_name(["ground", "max_slope"]) == "ground.max_slope"
    assert ids.normalise_param_name(["sensors", "d456_driver.topic"]) == "sensors.d456_driver.topic"
    with pytest.raises(ContractError):
        ids.normalise_param_name("")


@pytest.mark.parametrize("spelling", ["a..b", ".a.b", "a.b.", "..a..b..", ["a", "", "b"]])
def test_a_parameter_name_with_an_empty_component_is_refused(spelling):
    """Dropping the gap folded every one of these onto the one id `a.b`, so spellings no
    source agrees on shared a parameter and the one read last owned its value."""
    with pytest.raises(ContractError):
        ids.normalise_param_name(spelling)
    assert ids.normalise_param_name("a.b") == "a.b"


def test_the_name_source_precedence_is_the_model_vocabulary_and_is_order_independent():
    """`ids` imports nothing, so the members are spelled there. Two lists that drift is a
    write that silently ranks below everything."""
    from rosio import model

    assert set(ids.NAME_SOURCE_ORDER) == {member.value for member in model.NameSource}
    assert len(ids.NAME_SOURCE_ORDER) == len(model.NameSource)
    assert ids.NAME_SOURCE_ORDER[0] == model.NameSource.LITERAL.value
    assert ids.NAME_SOURCE_ORDER[-1] == model.NameSource.UNKNOWN.value

    for left in ids.NAME_SOURCE_ORDER:
        for right in ids.NAME_SOURCE_ORDER:
            assert ids.strongest_name_source(left, right) == ids.strongest_name_source(right, left)
    assert ids.strongest_name_source("literal", "argv") == "literal"
    assert ids.strongest_name_source("argv", "executable_default") == "argv"
    assert ids.strongest_name_source("observed", "unknown") == "observed"
    assert ids.strongest_name_source(None, "argv") == "argv"
    assert ids.strongest_name_source(None, None) is None
    assert ids.name_source_rank("a_member_from_a_later_minor") == len(ids.NAME_SOURCE_ORDER)
    assert ids.strongest_name_source("a_member_from_a_later_minor", "unknown") == "unknown"


@pytest.mark.parametrize(
    ("key", "fqn", "want"),
    [
        ("/**", "/anything/at_all", True),
        ("/**/ekf_local", "/ekf_local", True),
        ("/**/ekf_local", "/robot1/ekf_local", True),
        ("/**/ekf_local", "/ekf_global", False),
        ("/**/os_driver", "/ouster/os_driver", True),
        ("controller_server", "/controller_server", True),
        ("lifecycle_manager", "/lifecycle_manager_navigation", False),
        ("local_costmap/local_costmap", "/local_costmap/local_costmap", True),
    ],
)
def test_param_key_wildcards(key, fqn, want):
    assert ids.param_key_matches(key, fqn) is want


def test_digests_are_stable_and_field_order_free():
    assert ids.digest("a", "b") == ids.digest("a", "b")
    assert ids.digest("a", "b") != ids.digest("b", "a")
    assert ids.digest_fields({"a": 1, "b": [2, 3]}) == ids.digest_fields({"b": [2, 3], "a": 1})
    assert len(ids.role_digest(x=1)) == ids.DIGEST_LENGTH
    assert len(ids.edge_key(site="launch", profile="p")) == ids.DIGEST_LENGTH


def test_unresolved_ids_cannot_merge_two_sites():
    owner = ids.build_sym("nx_bringup", "src/motion_gate_bt_node.cpp", "MotionGate")
    first = ids.build_unresolved(owner, 'getInput("confirmation_topic")', "topic_name")
    second = ids.build_unresolved(owner, 'getInput("request_topic")', "topic_name")
    assert first != second
    assert ids.parse_unresolved(first) != ids.parse_unresolved(second)


def test_one_expression_at_two_sites_stays_two_blind_spots():
    """Same owner, same expression, two producing sites. One handle would keep the first
    evidence row and drop the second, reporting one blind spot where there are two."""
    owner = ids.build_node("/controller_server")
    expr = "self.get_parameter(name).value"
    file_id = ids.build_file("lx_bringup", "launch/nav_stack.launch.py")
    first = ids.build_unresolved(
        owner, expr, "topic_name", ids.unresolved_site(file_id, "plugin[a]")
    )
    second = ids.build_unresolved(
        owner, expr, "topic_name", ids.unresolved_site(file_id, "plugin[b]")
    )
    assert first != second


def test_an_unresolved_site_carries_no_line_hint():
    """A line hint is advisory, so a file gaining a line above the site must not move the
    handle."""
    file_id = ids.build_file("lx_bringup", "launch/nav_stack.launch.py")
    assert ids.unresolved_site(file_id, "plugin[a]") == f"{file_id}#plugin[a]"
    assert "12" not in ids.unresolved_site(file_id, "plugin[a]")


def test_a_blind_spot_with_no_site_keeps_its_handle():
    """Sites arrive collector by collector, and an entry that has none must not be
    renumbered by the ones that do."""
    owner = ids.build_node("/controller_server")
    assert ids.build_unresolved(owner, "expr", "topic_name") == ids.build_unresolved(
        owner, "expr", "topic_name", ""
    )


def test_two_argument_vectors_never_share_a_role_digest():
    """Members joined with a bare comma made ('sensor,--mode', 'fast') and ('sensor',
    '--mode,fast') one digest, so two nodes collapsed into one synthetic identity."""
    left = ids.role_digest(argv=("sensor,--mode", "fast"))
    right = ids.role_digest(argv=("sensor", "--mode,fast"))
    assert left != right


def test_a_sequence_member_holding_the_id_separator_is_framed():
    """The member escape is the one the id grammar already uses, so a member holding the
    separator cannot move a boundary."""
    assert ids.digest_fields({"argv": ("a|b", "c")}) != ids.digest_fields({"argv": ("a", "b|c")})
    assert ids.digest_fields({"argv": ("a%7Cb",)}) != ids.digest_fields({"argv": ("a|b",)})


def test_a_sequence_and_its_length_are_both_in_the_digest():
    assert ids.digest_fields({"argv": ("a", "b")}) != ids.digest_fields({"argv": ("a", "b", "")})
    assert ids.digest_fields({"argv": ()}) != ids.digest_fields({"argv": ("",)})


def test_a_part_holding_the_join_byte_is_not_two_parts():
    """digest joined its parts with a NUL, so one part spelling a NUL answered for two. A
    `-p x:="\\0"` on a command line reaches a digest through the candidate chain, so the
    separator is reachable content and not a byte no value can hold."""
    assert ids.digest("a", "b") != ids.digest("a\x00b")
    assert ids.digest("a\x00b", "c") != ids.digest("a", "b\x00c")
    assert ids.digest("", "") != ids.digest("\x00")


def test_a_part_boundary_survives_every_byte_a_part_can_hold():
    for filler in ("\x00", "|", "%", "1\x00", "\n", "0\x00"):
        assert ids.digest(f"a{filler}", "b") != ids.digest("a", f"{filler}b")
    assert ids.digest("ab") != ids.digest("a", "b")


def test_a_role_digest_is_stable_across_two_calls():
    fields = {"argv": ["--frame-id", "base_footprint"], "remaps": [], "ns": "/"}
    assert ids.digest_fields(fields) == ids.digest_fields(dict(reversed(list(fields.items()))))
