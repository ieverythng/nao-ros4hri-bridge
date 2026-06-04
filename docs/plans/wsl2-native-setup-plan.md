# WSL2 Native Setup Plan — nao-ros4hri-bridge

> Reviewed and improved version. Original plan at `docs/artifacts/wsl2-native-setup-plan.md`.

Generated: 2026-06-04
Branch: `feat/TFM-LLM_planner` (main), `feat/planner_llm_hooks` (chatbot_llm), `juan-feat-1` (dialogue_manager), `feat/TFM-LLM_planner` (planner_llm)

---

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
- Ollama running locally on `127.0.0.1:11434` with model `gpt-oss:20b-cloud`
- Hermes gateway at `http://127.0.0.1:8001/v1`

### What's NOT installed (hard blockers)
- ❌ ROS 2 Jazzy (`/opt/ros/jazzy` does not exist)
- ❌ colcon build tool
- ❌ SocialMinds apt repo
- ❌ Any `ros-jazzy-*` packages
- ❌ GStreamer audio stack
- ❌ Python deps: vosk, torch, ultralytics, textual, pydantic

### Source packages status (all 15 present in src/)
✅ kb_skills, dialogue_manager, asr_vosk, chatbot_llm, planner_common, planner_llm,
fake_skills, nao_say_skill, nao_replay_motion, nao_look_at, nao_orchestrator,
interaction_trace_viewer, nao_scene_grounding, nao_chatbot, simple_audio_capture

### Missing from src/ (need bootstrap or apt)
❌ interaction_skills — needed by nao_look_at, nao_orchestrator
❌ motions_skills — optional motion skill interfaces
✅ std_skills — already present in src/
❌ kb_msgs — reference-only clone OR SocialMinds apt package
❌ knowledge_core — reference-only clone OR SocialMinds apt package

---

## Phase 1: ROS 2 Jazzy Underlay (HARD BLOCKER)

```bash
# 1. Add ROS apt key and repo
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /etc/apt/keyrings/ros-archive.key

echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/ros-archive.key] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 2. Update and install ROS base + build tools
sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-ros-base \
  python3-colcon-common-extensions \
  python3-colcon-argcomplete

# 3. Install additional ROS packages from Dockerfile.full
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
  ros-jazzy-rqt-reconfigure

# 4. Install Python system packages from Dockerfile.full
sudo apt-get install -y \
  python3-opencv \
  python3-pil \
  python3-psutil \
  python3-requests \
  python3-scipy \
  python3-tqdm \
  python3-yaml \
  python3-gi \
  python3-matplotlib \
  python3-cpuinfo

# 5. Verify
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO   # should print "jazzy"
ros2 --version    # should work
```

> **Review finding:** Original plan missed `python3-colcon-argcomplete`, `python3-matplotlib`, and `python3-cpuinfo` from Dockerfile.full. Added above.

---

## Phase 2: SocialMinds Apt Repo (HARD BLOCKER)

```bash
# Add SocialMinds apt key and repo
curl -s https://socialminds.doc.iiia.csic.es/apt/public.key \
  | sudo gpg --dearmor -o /etc/apt/trusted.gpg.d/socialminds.gpg

echo "deb [signed-by=/etc/apt/trusted.gpg.d/socialminds.gpg] \
  https://socialminds.doc.iiia.csic.es/apt/ jazzy main" \
  | sudo tee /etc/apt/sources.list.d/socialminds.list

sudo apt-get update

# Install ALL SocialMinds packages from Dockerfile.full (20 packages)
sudo apt-get install -y \
  socialminds-ros-jazzy-audio-common \
  socialminds-ros-jazzy-std-skills \
  socialminds-ros-jazzy-chatbot-msgs \
  socialminds-ros-jazzy-hri-actions-msgs \
  socialminds-ros-jazzy-hri-face-detect-yunet \
  socialminds-ros-jazzy-kb-msgs \
  socialminds-ros-jazzy-knowledge-core \
  socialminds-ros-jazzy-hri-msgs \
  socialminds-ros-jazzy-hri-emotion-models \
  socialminds-ros-jazzy-hri-emotion-recognizer \
  socialminds-ros-jazzy-hri-person-manager \
  socialminds-ros-jazzy-hri-visualization \
  socialminds-ros-jazzy-expressive-face \
  socialminds-ros-jazzy-interaction-sim \
  socialminds-ros-jazzy-naoqi-bridge-msgs \
  socialminds-ros-jazzy-naoqi-driver \
  socialminds-ros-jazzy-launch-tui \
  socialminds-ros-jazzy-oro \
  socialminds-ros-jazzy-pyhri \
  socialminds-ros-jazzy-tts-msgs \
  socialminds-ros-jazzy-rqt-chat \
  socialminds-ros-jazzy-rqt-human-radar \
  socialminds-ros-jazzy-ros-qml-plugin \
  socialminds-ros-jazzy-ui-msgs \
  socialminds-ros-jazzy-ui-server
```

> **Review finding:** Original plan missed `hri-face-detect-yunet`, `hri-emotion-models`, `hri-emotion-recognizer`, `expressive-face`, `oro`, `rqt-human-radar`, `ros-qml-plugin`. All 25 SocialMinds packages from Dockerfile.full are now listed.

**⚠️ CRITICAL CHECK:** If the SocialMinds apt repo is unreachable or packages don't resolve,
we'll need to fall back to source builds using the bootstrap script's reference clones.

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
Run it to get missing packages into `src/` and `ref_src/knowledge_sources/`:

```bash
cd /home/juanbeck/Watson/repos/nao-ros4hri-bridge
chmod +x scripts/bootstrap_socialminds_sources.sh
./scripts/bootstrap_socialminds_sources.sh
```

This clones to `ref_src/knowledge_sources/` (reference only, NOT in build path):
- `kb_msgs` — from GitHub
- `knowledge_core` — from GitLab IIIA
- `interaction_sim` — from GitLab IIIA
- `oro` — from GitHub

This clones to `src/` (active workspace):
- `interaction_skills` — needed by nao_look_at, nao_orchestrator
- `std_skills` — already present, will update
- `motions_skills` — optional motion skill interfaces

> **Review finding:** Original plan incorrectly stated bootstrap clones kb_msgs and knowledge_core into src/. They go to ref_src/ only. The build depends on the SocialMinds apt packages for these, NOT the reference clones. If apt fails, you need to manually copy them into src/.

---

## Phase 5: Python Dependencies (venv)

```bash
cd /home/juanbeck/Watson/repos/nao-ros4hri-bridge
python3 -m venv .venv
source .venv/bin/activate

pip install --upgrade pip

# Core deps from Dockerfile.full
pip install vosk pyasyncore pyasynchat

# PyTorch CPU-only (for object detection if needed)
pip install --index-url https://download.pytorch.org/whl/cpu \
  torch torchvision

# Ultralytics (YOLO — for emorobcare_cv backend)
pip install --no-deps ultralytics ultralytics-thop

# Textual (launch TUI)
pip install "textual>=0.50,<1"

# Additional deps from package setup.py files
pip install pydantic
```

> **Review finding:** Original plan used `pip install` without `--break-system-packages` flag which is correct for venv, but missed that Dockerfile uses `--ignore-installed` for textual. Added explicit version constraint.

---

## Phase 6: Build the Workspace

```bash
source /opt/ros/jazzy/setup.bash
source /home/juanbeck/Watson/repos/nao-ros4hri-bridge/.venv/bin/activate
cd /home/juanbeck/Watson/repos/nao-ros4hri-bridge

# Build all packages (matches Dockerfile.full rebuild list)
colcon build --symlink-install --packages-up-to \
  kb_skills dialogue_manager asr_vosk chatbot_llm \
  planner_common planner_llm fake_skills nao_say_skill \
  nao_replay_motion nao_look_at nao_orchestrator \
  interaction_trace_viewer nao_scene_grounding \
  nao_chatbot simple_audio_capture

# Source the install space
source install/setup.bash
```

> **Review finding:** Original plan was missing `--packages-up-to` quoting — the package list needs to be a single quoted string or passed as separate args. Fixed above.

---

## Phase 7: LLM Backend Configuration

### Default settings (from stack_launch.py)

| Parameter | Default Value |
|---|---|
| `chatbot_server_url` | `http://10.7.138.215:8004/v1/chat/completions` |
| `planner_llm_base_url` | `http://10.7.138.215:8004` |
| `chatbot_model` | `QuantTrio/Qwen3-VL-30B-A3B-Instruct-AWQ` |
| `planner_llm_provider` | `openai_compatible` |

### Option A: Use local Ollama (already running on 127.0.0.1:11434)

Override launch args:
```
chatbot_server_url:=http://127.0.0.1:11434/api/chat
planner_llm_base_url:=http://127.0.0.1:11434
chatbot_model:=gpt-oss:20b-cloud
planner_llm_model:=gpt-oss:20b-cloud
```

> **Review finding:** Ollama uses `/api/chat` endpoint, NOT `/v1/chat/completions`. The chatbot_llm node expects an OpenAI-compatible endpoint. You may need to use the Hermes gateway instead which provides OpenAI-compatible API at `http://127.0.0.1:8001/v1`.

### Option B: Use Hermes gateway (recommended)

```
chatbot_server_url:=http://127.0.0.1:8001/v1/chat/completions
planner_llm_base_url:=http://127.0.0.1:8001/v1
chatbot_model:=qwen36-turbo-hermes
planner_llm_model:=qwen36-turbo-hermes
```

### Option C: Use IIIA ZeroTier LLM endpoint

If available, use the IIIA lab's vLLM instance over ZeroTier.

---

## Phase 8: Launch the Stack

```bash
source /opt/ros/jazzy/setup.bash
source /home/juanbeck/Watson/repos/nao-ros4hri-bridge/install/setup.bash

# Sim profile (no robot, fake skills, interaction sim)
ros2 launch nao_chatbot nao_chatbot_sim.launch.py \
  chatbot_server_url:=http://127.0.0.1:8001/v1/chat/completions \
  planner_llm_base_url:=http://127.0.0.1:8001/v1 \
  chatbot_model:=qwen36-turbo-hermes \
  planner_llm_model:=qwen36-turbo-hermes \
  start_naoqi_driver:=false \
  start_nao_robot:=false \
  start_fake_skills:=true \
  start_planner_llm:=true \
  chatbot_preflight_required:=false \
  planner_llm_preflight_required:=false
```

> **Review finding:** Preflight checks MUST be disabled for first run — the LLM endpoint needs to be verified working before enabling them. Also added explicit model names matching Hermes config.

---

## Known Issues / Risks for WSL2

### 1. nao_replay_motion — C++ node depends on naoqi_libqi
- This package has a C++ node (`nao_posture_bridge_node.cpp`) that links against `libqi`
- `ros-jazzy-naoqi-libqi` provides this from the ROS apt repo
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

### 6. Docker Desktop not running
- The `build_docker.sh` script requires Docker
- Native WSL2 approach bypasses this entirely — we install everything directly on the host
- **Not blocking** for native approach, but blocks Docker-based builds

> **Review finding:** Added Phase 6 (Docker status) as a known issue rather than a separate section. Clarified it's not blocking for native approach.

---

## Dependency Summary

| Category | Count | Status |
|----------|-------|--------|
| ROS 2 Jazzy base + extras | ~15 packages | ❌ Not installed |
| SocialMinds apt packages | 25 packages | ❌ Repo not configured |
| GStreamer audio stack | 5 packages | ❌ Not installed |
| Python pip packages | 8 packages | ❌ Not installed |
| Source-built packages | 15 | ✅ Ready to build |
| Bootstrap source repos | 4 ref + 3 src | ❌ Not cloned |
| Local stub packages | 6 (fake_skills, etc.) | ✅ Present in src/ |

**Total blockers to resolve before first build:** ROS 2 Jazzy + SocialMinds apt repo + colcon + bootstrap repos

---

## Recommended Order of Operations

1. **Install ROS 2 Jazzy** (Phase 1) — everything depends on this
2. **Add SocialMinds apt repo + install 25 packages** (Phase 2) — verify all resolve
3. **Install GStreamer + audio** (Phase 3)
4. **Bootstrap source repos** (Phase 4) — run the bootstrap script
5. **Set up Python venv + pip deps** (Phase 5)
6. **colcon build** (Phase 6) — expect and fix errors iteratively
7. **Configure LLM backend** (Phase 7) — Hermes gateway recommended
8. **Launch sim profile** (Phase 8) — with preflight disabled, fake skills enabled

---

## Quick Reference: Sim Launch Parameters

All overridable parameters for the sim profile:

```
chatbot_server_url          → LLM chat completions endpoint
planner_llm_base_url        → LLM base URL for planner
chatbot_model               → Model name for chatbot
planner_llm_model           → Model name for planner
start_fake_skills           → true (use fake skill providers)
start_planner_llm           → true (enable LLM planner)
chatbot_preflight_required  → false (disable for first run)
planner_llm_preflight_req.  → false (disable for first run)
start_naoqi_driver          → false (no robot)
start_nao_robot             → false (no robot)
start_rqt_chat              → true/false (GUI chat interface)
start_asr                   → true/false (voice input)
start_interaction_sim_perception → true/false (camera perception)
```
