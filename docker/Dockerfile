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
    gstreamer1.0-alsa \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-pulseaudio \
    gstreamer1.0-tools \
    python3-gi \
    python3-pip \
    socialminds-ros-jazzy-audio-common \
    socialminds-ros-jazzy-chatbot-msgs \
    socialminds-ros-jazzy-hri-actions-msgs \
    && rm -rf /var/lib/apt/lists/* && \
    python3 -m pip install --no-cache-dir --break-system-packages vosk

# Rebuild local workspace packages on top of the validated base.
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/ubuntu/ws && \
    rm -rf \
      /home/ubuntu/ws/build/std_skills \
      /home/ubuntu/ws/build/communication_skills \
      /home/ubuntu/ws/build/dialogue_manager \
      /home/ubuntu/ws/build/asr_vosk \
      /home/ubuntu/ws/build/nao_skills \
      /home/ubuntu/ws/build/nao_chatbot \
      /home/ubuntu/ws/build/simple_audio_capture \
      /home/ubuntu/ws/build/nao_skill_servers \
      /home/ubuntu/ws/install/std_skills \
      /home/ubuntu/ws/install/communication_skills \
      /home/ubuntu/ws/install/dialogue_manager \
      /home/ubuntu/ws/install/asr_vosk \
      /home/ubuntu/ws/install/nao_skills \
      /home/ubuntu/ws/install/nao_chatbot \
      /home/ubuntu/ws/install/simple_audio_capture \
      /home/ubuntu/ws/install/nao_skill_servers && \
    colcon build --symlink-install --packages-select std_skills communication_skills nao_skills dialogue_manager asr_vosk nao_chatbot simple_audio_capture nao_skill_servers"

# Keep entrypoint from this repo for deterministic sourcing behavior.
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
