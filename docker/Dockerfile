# docker/Dockerfile
FROM ros:jazzy-ros-base-noble

# Avoid interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Install build tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-vcstool \
    python3-colcon-common-extensions \
    python3-rosdep \
    git \
    wget \
    curl \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Clone naoqi_driver2 FIRST (it contains dependencies.repos)
RUN cd /ros2_ws/src && \
    git clone https://github.com/ros-naoqi/naoqi_driver2.git

# Use naoqi_driver2's dependencies.repos to clone everything else
RUN cd /ros2_ws/src && \
    vcs import < naoqi_driver2/dependencies.repos

# Copy YOUR custom source code
COPY src/ /ros2_ws/src/

# Initialize rosdep
RUN rosdep update || true

# Install ROS dependencies
# Using || true to continue even if some dependencies fail
RUN apt-get update && \
    rosdep install --from-paths /ros2_ws/src --ignore-src -r -y || true && \
    rm -rf /var/lib/apt/lists/*

# Agree to mesh licenses (non-interactive)
ENV I_AGREE_TO_NAO_MESHES_LICENSE=1
ENV I_AGREE_TO_PEPPER_MESHES_LICENSE=1

# Build workspace (skip meshes to avoid architecture issues)
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /ros2_ws && \
    colcon build --symlink-install \
        --packages-skip nao_meshes pepper_meshes \
        --continue-on-error \
        --cmake-args -DCMAKE_BUILD_TYPE=Release"

# Copy entrypoint
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
