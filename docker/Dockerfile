# XR-UAV-Swarm — ROS 2 backend image
#
# ROS 2 Jazzy (Ubuntu 24.04) + Crazyswarm2 (sim backend) + Crazyflie firmware
# Python bindings + rosbridge. Our own packages (ros2_ws/src) are NOT baked in:
# they are mounted from the repo at runtime so you can edit them live.
#
# Build (from repo root):   ./scripts/build_image.sh
FROM osrf/ros:jazzy-desktop

# Pinned versions — change these to update, then rebuild the image
ARG CRAZYSWARM2_REPO=https://github.com/IMRCLab/crazyswarm2.git
ARG CRAZYSWARM2_REF=f1e09954f56cd08e7d9aff5e13dc9a68795914a3
ARG CF_FIRMWARE_REPO=https://github.com/bitcraze/crazyflie-firmware.git
ARG CF_FIRMWARE_REF=e2fdc21159d40d2053fbfc92d14374996ffd2bd3

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ---- system + ROS packages -------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        git build-essential swig python3-dev python3-pip \
        libboost-program-options-dev libusb-1.0-0-dev \
        python3-scipy python3-yaml \
        ros-jazzy-rosbridge-suite \
        ros-jazzy-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# ---- Python deps for the Crazyswarm2 sim -----------------------------------
RUN pip3 install --no-cache-dir --break-system-packages --ignore-installed \
        "numpy<2" rowan nicegui cflib

# ---- Crazyflie firmware Python bindings (used by the sim backend) ----------
RUN git clone --recursive ${CF_FIRMWARE_REPO} /opt/crazyflie-firmware \
    && cd /opt/crazyflie-firmware \
    && git checkout ${CF_FIRMWARE_REF} \
    && git submodule update --init --recursive \
    && make cf2_defconfig \
    && make bindings_python \
    && python3 -c "import sys; sys.path.insert(0, 'build'); import cffirmware; print('cffirmware OK')"
ENV PYTHONPATH=/opt/crazyflie-firmware/build

# ---- Crazyswarm2 as an underlay workspace ----------------------------------
RUN mkdir -p /opt/crazyswarm_ws/src \
    && git clone ${CRAZYSWARM2_REPO} /opt/crazyswarm_ws/src/crazyswarm2 \
    && cd /opt/crazyswarm_ws/src/crazyswarm2 \
    && git checkout ${CRAZYSWARM2_REF} \
    && git submodule update --init --recursive \
    && apt-get update \
    && rosdep update --rosdistro jazzy \
    && rosdep install --from-paths /opt/crazyswarm_ws/src --ignore-src -y -r --rosdistro jazzy \
    && rm -rf /var/lib/apt/lists/* \
    && source /opt/ros/jazzy/setup.bash \
    && cd /opt/crazyswarm_ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && rm -rf build log

# ---- shell environment -----------------------------------------------------
RUN { \
      echo 'source /opt/ros/jazzy/setup.bash'; \
      echo 'source /opt/crazyswarm_ws/install/setup.bash'; \
      echo '[ -f /root/ros2_ws/install/setup.bash ] && source /root/ros2_ws/install/setup.bash'; \
      echo 'export PYTHONPATH=/opt/crazyflie-firmware/build:$PYTHONPATH'; \
    } >> /root/.bashrc

WORKDIR /root/ros2_ws
CMD ["bash"]
