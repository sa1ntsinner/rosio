#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
source /opt/demo_ws/install/setup.bash
exec "$@"
