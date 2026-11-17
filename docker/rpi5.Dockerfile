# Reproducibility check for Raspberry Pi 5 deployment (FR-HW-60).
# Build: scripts/deploy/build_arm64_container.sh
# This is not the delivery path; the Pi builds natively via rpi5_install.sh.
FROM ros:jazzy-ros-base

ENV DEBIAN_FRONTEND=noninteractive \
    FRET_ROOT=/opt/fret \
    ROS_DISTRO=jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-pip \
        python3-venv \
        python3-colcon-common-extensions \
        git \
        curl \
        ca-certificates \
        ros-jazzy-rmw-zenoh-cpp \
        v4l-utils \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /opt/fret/src
COPY . /opt/fret/src

# Minimal install path used by the container smoke (not a full rpi5_install).
RUN python3 -m venv --system-site-packages /opt/fret/venv \
    && /opt/fret/venv/bin/pip install --upgrade pip \
    && /opt/fret/venv/bin/pip install -e ".[hw,dev]" \
    && mkdir -p /opt/fret/releases/container /var/log/fret/diag /var/lib/fret

ENV PATH="/opt/fret/venv/bin:${PATH}" \
    PYTHONPATH="/opt/fret/src/src:${PYTHONPATH}"

CMD ["bash", "-lc", "uname -m && python3 -c 'import arco, fret; print(arco.__version__)'"]
