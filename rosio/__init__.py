"""rosio: a four layer atlas of a ROS 2 system, and the drift between the layers."""

__version__ = "0.1.0"

ATLAS_SCHEMA_VERSION = "1"

LAYERS = ("declared", "coded", "observed", "recorded")

__all__ = ["ATLAS_SCHEMA_VERSION", "LAYERS", "__version__"]
