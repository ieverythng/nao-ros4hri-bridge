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
COPY ref_src/interaction_skills/ /home/ubuntu/ws/src/interaction_skills/

# Most overlay rebuilds reuse a prepared base image, so only install runtime
# dependencies when they are genuinely missing.
RUN bash -lc 'set -euo pipefail; \
    packages=( \
      ros-jazzy-gscam \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-rosbridge-server \
      gstreamer1.0-alsa \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-pulseaudio \
      gstreamer1.0-tools \
      python3-gi \
      python3-pip \
      python3-yaml \
      ros-jazzy-rqt-console \
      ros-jazzy-rqt-gui \
      ros-jazzy-rqt-gui-py \
      ros-jazzy-rqt-image-view \
      ros-jazzy-rqt-reconfigure \
      socialminds-ros-jazzy-audio-common \
      socialminds-ros-jazzy-chatbot-msgs \
      socialminds-ros-jazzy-expressive-face \
      socialminds-ros-jazzy-hri-actions-msgs \
      socialminds-ros-jazzy-hri-emotion-models \
      socialminds-ros-jazzy-hri-emotion-recognizer \
      socialminds-ros-jazzy-hri-face-detect-yunet \
      socialminds-ros-jazzy-hri-msgs \
      socialminds-ros-jazzy-hri-person-manager \
      socialminds-ros-jazzy-hri-visualization \
      socialminds-ros-jazzy-interaction-sim \
      socialminds-ros-jazzy-kb-msgs \
      socialminds-ros-jazzy-knowledge-core \
      socialminds-ros-jazzy-oro \
      socialminds-ros-jazzy-pyhri \
      socialminds-ros-jazzy-rqt-chat \
      socialminds-ros-jazzy-rqt-human-radar \
      socialminds-ros-jazzy-ros-qml-plugin \
      socialminds-ros-jazzy-ui-msgs \
      socialminds-ros-jazzy-ui-server \
    ); \
    missing=(); \
    for pkg in "${packages[@]}"; do \
      if ! dpkg -s "${pkg}" >/dev/null 2>&1; then \
        missing+=("${pkg}"); \
      fi; \
    done; \
    if [ ! -x /opt/ros/jazzy/bin/rqt ]; then \
      missing+=(ros-jazzy-rqt-gui); \
    fi; \
    if [ "${#missing[@]}" -gt 0 ]; then \
      apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 update && \
      apt-get install -y --no-install-recommends "${missing[@]}" && \
      rm -rf /var/lib/apt/lists/*; \
    fi; \
    python3 -c "import importlib.util, sys; missing=[name for name in (\"vosk\", \"asyncore\", \"asynchat\") if importlib.util.find_spec(name) is None]; sys.exit(0 if not missing else 1)" \
      || python3 -m pip install --no-cache-dir --break-system-packages vosk pyasyncore pyasynchat'

# Rebuild the migrated local workspace packages on top of the validated base.
RUN bash -c "source /opt/ros/jazzy/setup.bash && \
    cd /home/ubuntu/ws && \
    rm -rf \
      /home/ubuntu/ws/build/std_skills \
      /home/ubuntu/ws/build/communication_skills \
      /home/ubuntu/ws/build/dialogue_manager \
      /home/ubuntu/ws/build/asr_vosk \
      /home/ubuntu/ws/build/chatbot_llm \
      /home/ubuntu/ws/build/interaction_skills \
      /home/ubuntu/ws/build/nao_skills \
      /home/ubuntu/ws/build/nao_chatbot \
      /home/ubuntu/ws/build/nao_look_at \
      /home/ubuntu/ws/build/nao_orchestrator \
      /home/ubuntu/ws/build/nao_replay_motion \
      /home/ubuntu/ws/build/nao_say_skill \
      /home/ubuntu/ws/build/simple_audio_capture \
      /home/ubuntu/ws/install/std_skills \
      /home/ubuntu/ws/install/communication_skills \
      /home/ubuntu/ws/install/dialogue_manager \
      /home/ubuntu/ws/install/asr_vosk \
      /home/ubuntu/ws/install/chatbot_llm \
      /home/ubuntu/ws/install/interaction_skills \
      /home/ubuntu/ws/install/nao_skills \
      /home/ubuntu/ws/install/nao_chatbot \
      /home/ubuntu/ws/install/nao_look_at \
      /home/ubuntu/ws/install/nao_orchestrator \
      /home/ubuntu/ws/install/nao_replay_motion \
      /home/ubuntu/ws/install/nao_say_skill \
      /home/ubuntu/ws/install/simple_audio_capture && \
    colcon build --symlink-install --packages-select \
      std_skills \
      communication_skills \
      interaction_skills \
      nao_skills \
      dialogue_manager \
      asr_vosk \
      chatbot_llm \
      nao_say_skill \
      nao_replay_motion \
      nao_look_at \
      nao_orchestrator \
      nao_chatbot \
      simple_audio_capture"

# Keep entrypoint from this repo for deterministic sourcing behavior.
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
