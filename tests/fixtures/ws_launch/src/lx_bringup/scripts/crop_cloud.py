#!/usr/bin/env python3
"""Stand in for a script started by ExecuteProcess with a --ros-args tail.

It is never executed by the tests. It exists so the argv in exec_process.launch.py
points at a file that is really in the workspace.
"""

import sys


def main() -> int:
    print("lx_bringup crop_cloud stub", sys.argv[1:])
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
