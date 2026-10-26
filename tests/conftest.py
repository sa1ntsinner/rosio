import os
import shutil
from pathlib import Path

import pytest

FIXTURES = Path(__file__).parent / "fixtures"

# A real workspace used as the test bench, named by the environment so that no path from
# one machine is written down here. Read only, tests must never write into it.
REFERENCE_WS_ENV = "ROSIO_REFERENCE_WS"


def _ros_available() -> bool:
    if os.environ.get("ROSIO_SKIP_ROS"):
        return False
    if not os.environ.get("AMENT_PREFIX_PATH"):
        return False
    try:
        import rclpy  # noqa: F401
    except ImportError:
        return False
    return True


def pytest_collection_modifyitems(config: pytest.Config, items: list) -> None:
    if _ros_available():
        return
    skip_ros = pytest.mark.skip(reason="no sourced ROS 2 environment")
    for item in items:
        if "ros" in item.keywords:
            item.add_marker(skip_ros)


@pytest.fixture(scope="session")
def fixtures_dir() -> Path:
    return FIXTURES


@pytest.fixture(scope="session")
def reference_ws() -> Path:
    """Path to the reference workspace, skipping the test when this machine has none.

    Set ROSIO_REFERENCE_WS to a workspace holding src/ and install/. Without it the suite
    runs everywhere else and only the reference tests are skipped.
    """
    configured = os.environ.get(REFERENCE_WS_ENV)
    if not configured:
        pytest.skip(f"{REFERENCE_WS_ENV} is not set")
    root = Path(configured)
    if not (root / "src").is_dir():
        pytest.skip(f"reference workspace not present at {root}")
    return root


@pytest.fixture
def workspace(tmp_path: Path) -> Path:
    """Empty writable workspace with the src and install layout rosio expects."""
    (tmp_path / "src").mkdir()
    (tmp_path / "install").mkdir()
    return tmp_path


@pytest.fixture
def copied_package(workspace: Path):
    """Copy a fixture package into a writable workspace and return its path."""

    def _copy(name: str) -> Path:
        source = FIXTURES / name
        if not source.is_dir():
            pytest.skip(f"fixture package {name} not present")
        target = workspace / "src" / name
        shutil.copytree(source, target)
        return target

    return _copy


@pytest.fixture
def out_dir(tmp_path: Path) -> Path:
    path = tmp_path / "rosio-out"
    path.mkdir()
    return path


@pytest.fixture(autouse=True)
def stable_env(monkeypatch: pytest.MonkeyPatch) -> None:
    """Pin the environment bits that could make output non deterministic.

    PYTHONHASHSEED is deliberately not set here: the interpreter reads it at start up, so
    setting it inside a running test changes nothing. The seed independence of the output is
    asserted by running a second interpreter with a different seed.
    """
    monkeypatch.setenv("TZ", "UTC")
    monkeypatch.delenv("ROSIO_CONFIG", raising=False)
