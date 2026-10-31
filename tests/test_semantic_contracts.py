"""Controls for scoped knowledge, including positive extraction prerequisites."""

from pathlib import Path

import pytest

from rosio import ids, model, schema
from rosio.drift.topics import type_key
from tests.test_eighth_trial import node, python_workspace
from tests.test_seventh_trial import findings


@pytest.mark.parametrize("spelling", ["cv_bridge::ROSCvMatContainer", "T::Msg", "Image"])
def test_a_custom_cpp_representation_does_not_claim_a_wire_type(spelling):
    assert type_key(spelling) is None


@pytest.mark.parametrize(
    "spelling", ["sensor_msgs/Image", "sensor_msgs/msg/Image", "sensor_msgs::msg::Image"]
)
def test_explicit_ros_message_spellings_identify_the_same_wire_type(spelling):
    assert type_key(spelling) == ("sensor_msgs", "Image")


def test_an_unread_active_provider_prevents_a_claim_of_absence(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {
            "server": node(
                "server", "        self.create_service(SetMode, 'lamp/enable', self.cb)\n"
            ),
            "client": node("client", "        self.create_client(SetMode, 'camera/enable')\n"),
        },
        'Node(package="external_camera", executable="driver", name="camera")',
    )
    assert ids.build_node("/camera") in atlas.entities
    calls = [edge for edge in atlas.edges.values() if edge.kind == model.EdgeKind.CALLS.value]
    assert any(edge.dst == ids.build_service("/camera/enable") for edge in calls)
    assert any(edge.kind == model.EdgeKind.SERVES.value for edge in atlas.edges.values())
    assert findings(atlas, "service.no_server") == []


def test_unread_code_in_another_profile_does_not_hide_a_known_violation(tmp_path: Path):
    atlas = python_workspace(
        tmp_path,
        {
            "server": node("server", "        self.create_service(SetMode, 'mode', self.cb)\n"),
            "client": node("client", "        self.create_client(SetMode, 'mod')\n"),
        },
    )
    (before,) = findings(atlas, "service.no_server")
    builder = model.AtlasBuilder()
    builder.atlas = atlas
    other = builder.add_profile("other")
    camera = ids.build_node("/camera")
    builder.node(camera, name_source="literal")
    builder.fact(camera, "declared", profile=other, values={"pkg": "external_camera"})
    builder.recompute()
    assert schema.validate(atlas.to_json()) == []
    (after,) = findings(atlas, "service.no_server")
    assert after.id == before.id
