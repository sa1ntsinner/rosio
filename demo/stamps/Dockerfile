FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update \
 && apt-get install -y --no-install-recommends python3-pip \
 && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/demo_ws

COPY stamplint src/stamplint
COPY stampdoctor src/stampdoctor

# the pip shipped with 22.04 builds a pyproject only package as UNKNOWN 0.0.0
RUN pip3 install --no-cache-dir --upgrade pip \
 && pip3 install --no-cache-dir ./src/stamplint

RUN source /opt/ros/humble/setup.bash \
 && colcon build --packages-select stampdoctor \
 && rm -rf build log

COPY demo/publishers.py demo/entrypoint.sh /opt/demo/
RUN chmod +x /opt/demo/entrypoint.sh

ENTRYPOINT ["/opt/demo/entrypoint.sh"]
CMD ["ros2", "run", "stampdoctor", "check", "--duration", "10"]
