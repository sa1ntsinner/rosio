"""The header inventory keeps library include paths and measured ownership."""

from corpus.distros import collect


def test_library_headers_keep_the_paths_used_by_includes(monkeypatch):
    class Listing:
        stdout = """package rclcpp
package behaviortree_cpp_v3
package libceres-dev
package libopencv-dev
package libpcl-dev
header rclcpp/rclcpp/node.hpp
header behaviortree_cpp_v3/bt_factory.h
system_header libceres-dev ceres/local_parameterization.h
system_header libopencv-dev opencv2/core.hpp
system_header libpcl-dev pcl/point_cloud.h
"""

    monkeypatch.setattr(collect, "measuring_image", lambda distro: "image")
    monkeypatch.setattr(collect.subprocess, "run", lambda *a, **k: Listing())
    measured = collect.image_listing("humble")
    assert measured["headers"] == {
        "rclcpp/node.hpp": "rclcpp",
        "behaviortree_cpp_v3/bt_factory.h": "behaviortree_cpp_v3",
        "ceres/local_parameterization.h": "libceres-dev",
        "opencv2/core.hpp": "libopencv-dev",
        "pcl/point_cloud.h": "libpcl-dev",
    }


def test_prefix_successors_are_measured_even_when_present_everywhere(monkeypatch):
    monkeypatch.setattr(collect, "released", lambda distro: {"behaviortree_cpp"})

    def image(distro):
        old = distro == "humble"
        prefix = "behaviortree_cpp_v3" if old else "behaviortree_cpp"
        return {
            "packages": {prefix},
            "headers": {f"{prefix}/bt_factory.h": prefix},
            "deprecated": set(),
            "toolchain": {},
        }

    monkeypatch.setattr(collect, "image_listing", image)
    facts = collect.collect()["headers"]
    assert facts["behaviortree_cpp_v3/bt_factory.h"] == {
        "package": "behaviortree_cpp_v3",
        "present": ["humble"],
        "checked": list(collect.DISTROS),
    }
    assert facts["behaviortree_cpp/bt_factory.h"]["present"] == list(collect.DISTROS[1:])


def test_an_uninstalled_released_package_does_not_prove_header_removal(monkeypatch):
    monkeypatch.setattr(
        collect, "released", lambda distro: {"behaviortree_cpp", "behaviortree_cpp_v3"}
    )

    def image(distro):
        prefix = "behaviortree_cpp_v3" if distro == "humble" else "behaviortree_cpp"
        return {
            "packages": {prefix},
            "headers": {f"{prefix}/bt_factory.h": prefix},
            "deprecated": set(),
            "toolchain": {},
        }

    monkeypatch.setattr(collect, "image_listing", image)
    assert "behaviortree_cpp_v3/bt_factory.h" not in collect.collect()["headers"]
