# docker/Dockerfile
#
# Canonical overlay strategy:
# - Reuse a validated runtime image (default: iiia:nao)
# - Overlay repository sources (src/) and rebuild local packages
# - The base image is still expected to provide the validated NAOqi runtime
#   stack; this overlay only refreshes the repo-owned packages on top of it.
ARG BASE_IMAGE=iiia:nao
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /home/ubuntu/ws

# Most overlay rebuilds reuse a prepared base image, so only install runtime
# dependencies when they are genuinely missing. Keep this before copying sources
# so ordinary code edits do not force apt/pip network checks on every rebuild.
RUN bash -lc 'set -euo pipefail; \
    packages=( \
      ros-jazzy-cv-bridge \
      ros-jazzy-gscam \
      ros-jazzy-image-transport-plugins \
      ros-jazzy-rosbridge-server \
      curl \
      gstreamer1.0-alsa \
      gstreamer1.0-plugins-base \
      gstreamer1.0-plugins-good \
      gstreamer1.0-pulseaudio \
      gstreamer1.0-tools \
      python3-colcon-common-extensions \
      python3-cpuinfo \
      python3-gi \
      python3-matplotlib \
      python3-opencv \
      python3-pil \
      python3-pip \
      python3-psutil \
      python3-requests \
      python3-scipy \
      python3-tqdm \
      python3-yaml \
      ros-jazzy-rqt-console \
      ros-jazzy-rqt \
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
      socialminds-ros-jazzy-launch-tui \
      socialminds-ros-jazzy-oro \
      socialminds-ros-jazzy-pyhri \
      socialminds-ros-jazzy-rqt-chat \
      socialminds-ros-jazzy-rqt-human-radar \
      socialminds-ros-jazzy-ros-qml-plugin \
      socialminds-ros-jazzy-ui-msgs \
      socialminds-ros-jazzy-ui-server \
      zstd \
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
    python3 -c "import importlib.util, sys; missing=[name for name in (\"vosk\", \"asyncore\", \"asynchat\", \"ultralytics\", \"torch\", \"torchvision\") if importlib.util.find_spec(name) is None]; sys.exit(0 if not missing else 1)" \
      || ( \
        python3 -m pip install --no-cache-dir --break-system-packages vosk pyasyncore pyasynchat && \
        python3 -m pip install --no-cache-dir --break-system-packages \
          --index-url https://download.pytorch.org/whl/cpu \
          torch torchvision && \
        python3 -m pip install --no-cache-dir --break-system-packages --no-deps \
          ultralytics ultralytics-thop \
      ); \
    # launch_tui requires Textual >=0.50,<1. Reinstall only when needed so
    # overlay builds still work in offline/restricted environments.
    if ! python3 -c "import importlib.metadata, importlib.util, sys; \
spec = importlib.util.find_spec(\"textual\"); \
parts = importlib.metadata.version(\"textual\").split(\".\") if spec else []; \
major = int(parts[0]) if parts and parts[0].isdigit() else -1; \
minor = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else -1; \
sys.exit(0 if spec and major == 0 and minor >= 50 else 1)"; \
    then \
      python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
        "textual>=0.50,<1"; \
    fi'

# Demo profiles can launch split chatbot/planner Ollama servers from ROS launch.
# Keep this before copying sources so normal overlay rebuilds reuse the binary.
RUN bash -lc 'set -euo pipefail; \
    if ! command -v ollama >/dev/null 2>&1; then \
      curl -fsSL https://ollama.com/install.sh | sh; \
    fi'

# Remove base-image source tree first so deleted files are not kept as stale leftovers.
RUN rm -rf /home/ubuntu/ws/src && mkdir -p /home/ubuntu/ws/src

# Overlay repository sources into the runtime workspace. External packages kept
# out of the monorepo history, such as interaction_skills and the emorobcare
# detector stack, are copied too when they are present under src/.
COPY src/ /home/ubuntu/ws/src/

# Rebuild the migrated local workspace packages on top of the validated base.
RUN bash -lc 'source /opt/ros/jazzy/setup.bash && \
    cd /home/ubuntu/ws && \
    rebuild_packages=( \
      kb_skills \
      chatbot_msgs \
      dialogue_manager \
      rqt_dialogues \
      asr_vosk \
      chatbot_llm \
      planner_common \
      planner_llm \
      fake_skills \
      nao_say_skill \
      nao_replay_motion \
      nao_look_at \
      nao_orchestrator \
      interaction_trace_viewer \
      nao_scene_grounding \
      nao_chatbot \
      simple_audio_capture \
    ); \
    for optional_pkg in emorobcare_cv_msgs emorobcare_cv_object_detection my_game_interface; do \
      if [ -d "/home/ubuntu/ws/src/${optional_pkg}" ]; then \
        rebuild_packages+=("${optional_pkg}"); \
      fi; \
    done; \
    for pkg in "${rebuild_packages[@]}"; do \
      rm -rf "/home/ubuntu/ws/build/${pkg}" "/home/ubuntu/ws/install/${pkg}"; \
    done; \
    colcon build --symlink-install --packages-up-to "${rebuild_packages[@]}"'

# launch_tui is required by some operator flows; refresh it only when missing so
# overlay builds keep working when apt mirrors are temporarily unreachable.
RUN bash -lc 'set -euo pipefail; \
    if ! dpkg -s socialminds-ros-jazzy-launch-tui >/dev/null 2>&1; then \
      apt-get -o Acquire::Retries=3 -o Acquire::http::Timeout=30 -o Acquire::https::Timeout=30 update && \
      apt-get install -y --no-install-recommends socialminds-ros-jazzy-launch-tui && \
      rm -rf /var/lib/apt/lists/*; \
    fi; \
    if ! python3 -c "import importlib.metadata, importlib.util, sys; \
spec = importlib.util.find_spec(\"textual\"); \
parts = importlib.metadata.version(\"textual\").split(\".\") if spec else []; \
major = int(parts[0]) if parts and parts[0].isdigit() else -1; \
minor = int(parts[1]) if len(parts) > 1 and parts[1].isdigit() else -1; \
sys.exit(0 if spec and major == 0 and minor >= 50 else 1)"; \
    then \
      python3 -m pip install --no-cache-dir --break-system-packages --ignore-installed \
        "textual>=0.50,<1"; \
    fi'

# Keep entrypoint from this repo for deterministic sourcing behavior.
COPY docker/ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
