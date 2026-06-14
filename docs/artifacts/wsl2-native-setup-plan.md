# WSL2 Native Setup Plan — nao-ros4hri-bridge

Generated: 2026-06-05
Branch: `feat/TFM-LLM_planner` (main), `feat/planner_llm_hooks` (chatbot_llm), `juan-feat-1` (dialogue_manager)

## Executive Summary

The stack currently runs in Docker with a full SocialMinds apt repo + ROS 2 Jazzy underlay.
To run natively in WSL2, we need to replicate that environment: install ROS 2 Jazzy, add the
SocialMinds apt repo, bootstrap missing source packages, and configure an LLM backend.

**Estimated effort:** 1-2 hours of setup + debugging, assuming the SocialMinds apt repo is reachable.

---

## Current State

### What's installed
- Ubuntu 24.04.3 LTS (amd64) in WSL2
- Python 3.11.15 (system), pip3 bound to Python 3.12 (mismatch)
- Ollama running locally on `127.0.0.1:11434`
- Hermes gateway configured with external LLM endpoints

### What's NOT installed (hard blockers)
- ❌ ROS 2 Jazzy (`/opt/ros/jazzy` does not exist)
- ❌ colcon build tool
- ❌ SocialMinds apt repo
- ❌ Any `ros-jazzy-*` packages
- ❌ GStreamer audio stack
- ❌ Python deps: vosk, torch, ultralytics, textual, pydantic

---

## Phase 1: ROS 2 Jazzy Underlay (HARD BLOCKER)

```bash
# 1. Add ROS apt key and repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /etc/apt/keyrings/ros-archive.key

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive.key] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Update and install ROS base
sudo apt-get update
sudo apt-get install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-colcon-ros

# 3. Verify
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO  # should print "jazzy"
ros2 --version   # should work
```

### Additional ROS packages needed (from Dockerfile.full)
```bash
sudo apt-get install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-gscam \
  ros-jazzy-image-transport-plugins \
  ros-jazzy-rosbridge-server \
  ros-jazzy-naoqi-libqi \
  ros-jazzy-rqt \
  ros-jazzy-rqt-console \
  ros-jazzy-rqt-gui \
  ros-jazzy-rqt-gui-py \
  ros-jazzy-rqt-image-view \
  ros-jazzy-rqt-reconfigure \
  ros-jazzy-diagnostic-aggregator \
  python3-colcon-argcomplete \
  python3-ros-industrial-collada-urdf \
  python3-yaml \
  python3-requests \
  python3-opencv \
  python3-pil \
  python3-psutil \
  python3-scipy \
  python3-tqdm \
  python3-gi
```

---

## Phase 2: SocialMinds Apt Repo (HARD BLOCKER)

```bash
# Add SocialMinds apt key and repo
sudo apt-get install -y curl gnupg
curl -s https://socialminds.doc.iiia.csic.es/apt/public.key \
  | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/socialminds.gpg

echo "deb [signed-by=/etc/apt/trusted.gpg.d/socialminds.gpg] \
  https://socialminds.doc.iiia.csic.es/apt/ jazzy main" \
  | sudo tee /etc/apt/sources.list.d/socialminds.list

sudo apt-get update

# Install SocialMinds packages from Dockerfile.full
sudo apt-get install -y \
  socialminds-ros-jazzy-audio-common \
  socialminds-ros-jazzy-std-skills \
  socialminds-ros-jazzy-chatbot-msgs \
  socialminds-ros-jazzy-hri-actions-msgs \
  socialminds-ros-jazzy-kb-msgs \
  socialminds-ros-jazzy-knowledge-core \
  socialminds-ros-jazzy-hri-msgs \
  socialminds-ros-jazzy-hri-person-manager \
  socialminds-ros-jazzy-hri-visualization \
  socialminds-ros-jazzy-naoqi-bridge-msgs \
  socialminds-ros-jazzy-naoqi-driver \
  socialminds-ros-jazzy-launch-tui \
  socialminds-ros-jazzy-pyhri \
  socialminds-ros-jazzy-tts-msgs \
  socialminds-ros-jazzy-rqt-chat \
  socialminds-ros-jazzy-interaction-sim \
  socialminds-ros-jazzy-ui-server \
  socialminds-ros-jazzy-ui-msgs
```

**⚠️ CRITICAL CHECK:** If the SocialMinds apt repo is unreachable or packages don't resolve,
we'll need to fall back to source builds. See Phase 4.

---

## Phase 3: GStreamer + Audio Stack

```bash
sudo apt-get install -y \
  gstreamer1.0-alsa \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-pulseaudio \
  gstreamer1.0-tools
```

---

## Phase 4: Bootstrap Missing Source Packages

The bootstrap script at `scripts/bootstrap_socialminds_sources.sh` clones reference repos.
Run it to get `interaction_skills`, `std_skills`, `motions_skills` into `src/`:

```bash
cd /home/juanbeck/Watson/repos/nao-ros4hri-bridge
chmod +x scripts/bootstrap_socialminds_sources.sh
./scripts/bootstrap_socialminds_sources.sh
```

This clones:
- `interaction_skills` → needed by nao_look_at, nao_orchestrator
- `std_skills` → needed by communication_skills, nao_say_skill, etc.
- `motions_skills` → optional motion skill interfaces

**Packages NOT cloned by bootstrap (need SocialMinds apt or manual resolution):**
- `kb_msgs` — from `socialminds-ros-jazzy-kb-msgs` OR clone from GitHub
- `knowledge_core` — from `socialminds-ros-jazzy-knowledge-core` OR clone from GitLab
- `skill_common` — NOT in bootstrap script; likely a transitive dep of std_skills or SocialMinds apt
- `launch_pal` — NOT in bootstrap; check if it's a SocialMinds package or ROS standard

---

## Phase 5: Python Dependencies (venv)

```bash
cd /home/juanbeck/Watson/repos/nao-ros4hri-bridge
python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip
pip install \
  vosk \
  pyasyncore \
  pyasynchat \
  pydantic \
  "textual>=0.50,<1"

# PyTorch CPU-only (for object detection if needed)
pip install --index-url https://download.pytorch.org/whl/cpu \
  torch torchvision

# Ultralytics (YOLO — for emorobcare_cv backend)
pip install --no-deps ultralytics ultralytics-thop
```

---

## Phase 6: Build the Workspace

```bash
source /opt/ros/jazzy/setup.bash
source /home/juanbeck/Watson/repos/nao-ros4hri-bridge/.venv/bin/activate
cd /home/juanbeck/Watson/repos/nao-ros4hri-bridge

# Build all packages
colcon build --symlink-install --packages-up-to \
  kb_skills dialogue_manager asr_vosk chatbot_llm \
  planner_common planner_llm fake_skills nao_say_skill \
  nao_replay_motion nao_look_at nao_orchestrator \
  interaction_trace_viewer nao_scene_grounding \
  nao_chatbot simple_audio_capture

# Source the install space
source install/setup.bash
```

---

## Phase 7: LLM Backend Configuration

The sim profile defaults to an external vLLM endpoint at `http://10.7.138.215:8004`.
Options for local WSL2:

### Option A: Use local Ollama (already running on 127.0.0.1:11434)
Override launch args:
```
chatbot_server_url:=http://127.0.0.1:11434/api/chat
planner_llm_base_url:=http://127.0.0.1:11434
chatbot_model:=<ollama-model-name>
planner_llm_model:=<ollama-model-name>
```

### Option B: Use Hermes gateway (http://127.0.0.1:8001/v1)
Override launch args to point to the Hermes OpenAI-compatible endpoint.

### Option C: Use IIIA ZeroTier LLM endpoint
You mentioned an IIIA-owned endpoint — provide the URL and we'll configure it.

---

## Phase 8: Launch the Stack

```bash
source /opt/ros/jazzy/setup.bash
source /home/juanbeck/Watson/repos/nao-ros4hri-bridge/install/setup.bash

# Sim profile (no robot, fake skills, interaction sim)
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  chatbot_server_url:=http://127.0.0.1:11434/api/chat \
  planner_llm_base_url:=http://127.0.0.1:11434 \
  chatbot_model:=qwen2.5:7b \
  planner_llm_model:=qwen2.5:7b \
  start_naoqi_driver:=false \
  start_nao_robot:=false \
  start_fake_skills:=true \
  start_planner_llm:=true \
  chatbot_preflight_required:=false \
  planner_llm_preflight_required:=false
```

---

## Known Issues / Risks for WSL2

### 1. nao_replay_motion — C++ node depends on naoqi_libqi
- This package has a C++ node (`nao_posture_bridge_node.cpp`) that links against `libqi`
- `ros-jazzy-naoqi-libqi` provides this, but it's designed for NAO robot SDK
- In sim mode, the posture bridge tries to connect to a robot IP — will fail gracefully?
- **Risk:** May crash on startup if naoqi_libqi isn't properly linked
- **Mitigation:** Disable with `start_nao_replay_motion:=false` initially

### 2. Camera access in WSL2
- The interaction sim perception layer uses gscam with `/dev/video0`
- WSL2 camera access requires Windows host USB passthrough or WSLg
- **Risk:** gscam node may fail to open `/dev/video0`
- **Mitigation:** Disable with `start_interaction_sim_perception:=false`

### 3. GUI tools (rqt, rviz) in WSL2
- Require X11/Wayland forwarding via WSLg
- rqt_chat is used for the dialogue interface
- **Risk:** GUI apps may not render properly
- **Mitigation:** Disable with `start_rqt_chat:=false`, use CLI instead

### 4. Audio capture (simple_audio_capture)
- Uses GStreamer + PulseAudio for microphone input
- WSL2 audio support is limited (WSLg PulseAudio forwarding)
- **Risk:** Microphone may not be accessible
- **Mitigation:** Disable ASR with `start_asr:=false`

### 5. LLM preflight checks
- Sim profile has `chatbot_preflight_required=true` and `planner_llm_preflight_required=true`
- If the LLM endpoint is unreachable, nodes won't activate
- **Mitigation:** Override to `false` for initial testing

---

## Dependency Summary

| Category | Count | Status |
|----------|-------|--------|
| ROS 2 Jazzy base | — | ❌ Not installed |
| SocialMinds apt packages | ~20 | ❌ Repo not configured |
| Standard ROS apt packages | ~15 | ❌ Not installed |
| GStreamer audio stack | 5 | ❌ Not installed |
| Python pip packages | 8 | ❌ Not installed |
| Source-built packages | 18 | ✅ Ready to build |
| Bootstrap source repos | 4 | ❌ Not cloned |
| Local stub packages | 6 | ✅ Present in src/ |

**Total blockers to resolve before first build:** ROS 2 Jazzy + SocialMinds apt repo + colcon + bootstrap repos

---

## Recommended Order of Operations

1. **Install ROS 2 Jazzy** (Phase 1) — everything depends on this
2. **Add SocialMinds apt repo** (Phase 2) — verify packages resolve
3. **Install GStreamer + audio** (Phase 3)
4. **Bootstrap source repos** (Phase 4) — run the bootstrap script
5. **Set up Python venv + pip deps** (Phase 5)
6. **colcon build** (Phase 6) — expect and fix errors iteratively
7. **Configure LLM backend** (Phase 7) — decide on Ollama vs Hermes vs IIIA endpoint
8. **Launch sim profile** (Phase 8) — with preflight disabled, fake skills enabled

---

## Docker Status

Docker Desktop is not running. The build_docker.sh script requires Docker.
Native WSL2 approach bypasses this entirely — we install everything directly on the host.

If you want to use Docker later, start Docker Desktop and ensure WSL integration is enabled:
```bash
# On Windows side
docker-desktop.exe wsl integrate enable --distribution Ubuntu
```
