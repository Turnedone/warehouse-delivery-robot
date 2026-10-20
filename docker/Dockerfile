# Development and CI image: ROS 2 Jazzy desktop + Gazebo Harmonic + Nav2 + everything the
# workspace's package.xml files ask for (installed by rosdep, so this file never drifts from them).
#
#   docker build -f docker/Dockerfile -t wdr:dev --target deps .     dependencies only (dev container)
#   docker build -f docker/Dockerfile -t wdr .                        + the workspace, built
#   docker run --rm wdr bash -lc "colcon test --event-handlers console_cohesion+ && colcon test-result"
#
# Runs headless by default. For Gazebo's or RViz's windows, forward a display (see .devcontainer).

ARG ROS_DISTRO=jazzy
FROM osrf/ros:${ROS_DISTRO}-desktop AS deps
ARG ROS_DISTRO
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=${ROS_DISTRO} \
    RCUTILS_COLORIZED_OUTPUT=1

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip python3-pytest python3-colcon-common-extensions git gdb \
    && rm -rf /var/lib/apt/lists/*

# Only the package manifests first, so the dependency layer is cached until a package.xml changes
WORKDIR /ws
COPY src/wdr_interfaces/package.xml src/wdr_interfaces/
COPY src/wdr_sim/package.xml src/wdr_sim/
COPY src/wdr_mission/package.xml src/wdr_mission/
COPY src/wdr_bringup/package.xml src/wdr_bringup/
COPY src/wdr_tools/package.xml src/wdr_tools/
RUN apt-get update \
    && rosdep update --rosdistro ${ROS_DISTRO} \
    && rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} \
    && rm -rf /var/lib/apt/lists/*

# Same lint tool versions as CI
RUN pip3 install --break-system-packages clang-format==23.1.3 clang-tidy==23.1.3

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc \
    && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc

FROM deps AS build
COPY src /ws/src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
       --event-handlers console_cohesion+
CMD ["bash", "-l"]
