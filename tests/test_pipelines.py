from pathlib import Path

from rosio import model
from rosio.collect import CollectContext, entrypoints
from rosio.config import Config, RootSpec
from rosio.workspace import Workspace


def test_yaml_helpers_are_not_parameter_pipelines(tmp_path: Path):
    (tmp_path / "helpers.py").write_text("""
import yaml
import tempfile

def pose(value):
    result = yaml.safe_load(value)
    result['x'] = 0.0
    return result

def rewrite(filename, root_key):
    data = yaml.safe_load(open(filename))
    data[root_key] = data
    out = tempfile.NamedTemporaryFile(mode='w', delete=False)
    yaml.safe_dump(data, out)
    return out.name
""")
    roots = (RootSpec("src", str(tmp_path)),)
    ws = Workspace.discover(roots)
    ctx = CollectContext(ws=ws, cfg=Config(roots=roots, profiles=()), builder=model.AtlasBuilder())
    entrypoints.EntrypointsCollector().collect(ctx)
    assert ctx.declared.pipelines == []
    assert ctx.builder.build().unresolved == []
