"""Case 15: a stand in for nav2_common RewrittenYaml wrapped in a ParameterFile stand in.

The two classes below carry the same duck type as the nav2_common pair and write a temp
file at evaluation time, exactly like the real ones. The resolver must intercept them and
keep provenance on source_file plus the rewrite list, never on the temp path.

Three traps are reproduced.
  1. A rewrite of a key the source file does not contain (use_sim_time) is a silent no op.
  2. A rewrite is applied at every depth the bare key occurs, not only at node level.
  3. convert_types coerces by string inspection, so rewriting a string valued parameter
     with a value containing a dot changes its type to float.

Both public and name mangled attribute spellings are exposed, because nav2_common keeps
these values in name mangled privates and a duck typed describe must accept either.
"""

import os
import tempfile

import launch
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node

_CONFIG = os.path.normpath(
    os.path.join(os.path.dirname(os.path.realpath(__file__)), os.pardir, "config")
)


class LxRewrittenYaml(launch.Substitution):
    """Same shape as nav2_common.launch.RewrittenYaml, minus the substitution plumbing."""

    def __init__(self, source_file, param_rewrites, root_key="", convert_types=False):
        super().__init__()
        self.source_file = source_file
        self.param_rewrites = dict(param_rewrites)
        self.root_key = root_key
        self.convert_types = convert_types
        self._RewrittenYaml__source_file = source_file
        self._RewrittenYaml__param_rewrites = self.param_rewrites
        self._RewrittenYaml__root_key = root_key
        self._RewrittenYaml__convert_types = convert_types

    @property
    def name(self):
        return self.source_file

    def describe(self):
        return ""

    def perform(self, context):
        with open(self.source_file) as handle:
            data = yaml.safe_load(handle)
        _rewrite(data, self.param_rewrites, self.convert_types)
        if self.root_key:
            data = {self.root_key: data}
        target = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        yaml.safe_dump(data, target)
        target.close()
        return target.name


class LxParameterFile(launch.Substitution):
    """Same shape as launch_ros.descriptions.ParameterFile."""

    def __init__(self, param_file, allow_substs=False):
        super().__init__()
        self.param_file = param_file
        self.allow_substs = allow_substs

    def describe(self):
        return ""

    def perform(self, context):
        return self.param_file.perform(context)


def _rewrite(node, rewrites, convert_types):
    if not isinstance(node, dict):
        return
    for key, value in node.items():
        if isinstance(value, dict):
            _rewrite(value, rewrites, convert_types)
        elif key in rewrites:
            node[key] = _convert(rewrites[key]) if convert_types else rewrites[key]


def _convert(text):
    try:
        return float(text) if "." in text else int(text)
    except ValueError:
        pass
    if text.lower() in ("true", "false"):
        return text.lower() == "true"
    return text


def generate_launch_description():
    source = os.path.join(_CONFIG, "rewrite_source.yaml")
    rewrites = {
        "anchor_tag": "2.5",
        "autostart": "false",
        "use_sim_time": "true",
    }
    plain = LxParameterFile(
        LxRewrittenYaml(
            source_file=source,
            param_rewrites=rewrites,
            root_key="",
            convert_types=True,
        ),
        allow_substs=True,
    )
    wrapped = LxParameterFile(
        LxRewrittenYaml(
            source_file=source,
            param_rewrites=rewrites,
            root_key="robot1",
            convert_types=True,
        ),
        allow_substs=True,
    )
    return LaunchDescription(
        [
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="rewritten_node",
                parameters=[plain, {"use_sim_time": True}],
            ),
            Node(
                package="lx_bringup",
                executable="lx_controller",
                name="rewritten_node",
                namespace="robot1",
                parameters=[wrapped],
            ),
        ]
    )
