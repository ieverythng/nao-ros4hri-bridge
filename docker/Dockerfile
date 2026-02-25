# docker/Dockerfile
#
# Canonical overlay strategy:
# - Reuse a validated runtime image (default: iiia:nao)
# - Overlay repository sources (src/) and rebuild local packages
ARG BASE_IMAGE=iiia:nao
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/ubuntu/ws

# Remove base-image source tree first so deleted files are not kept as stale leftovers.
RUN rm -rf /home/ubuntu/ws/src && mkdir -p /home/ubuntu/ws/src

# Overlay repository sources into the runtime workspace.
COPY src/ /home/ubuntu/ws/src/

# ASR optional runtime dependencies for laptop microphone mode.
RUN apt-get update && apt-get install -y --no-install-recommends \
    libportaudio2 \
    python3-pip \
    && rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install --no-cache-dir --break-system-packages sounddevice vosk

# Rebuild local workspace packages on top of the validated base.
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/ubuntu/ws && \
    rm -rf \
      /home/ubuntu/ws/build/nao_skills \
      /home/ubuntu/ws/build/nao_chatbot \
      /home/ubuntu/ws/build/nao_posture_bridge \
      /home/ubuntu/ws/install/nao_skills \
      /home/ubuntu/ws/install/nao_chatbot \
      /home/ubuntu/ws/install/nao_posture_bridge && \
    colcon build --symlink-install --packages-select nao_skills nao_chatbot nao_posture_bridge"

# Keep entrypoint from this repo for deterministic sourcing behavior.
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
