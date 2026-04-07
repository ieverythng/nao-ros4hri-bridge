#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REF_SRC_DIR="${ROOT_DIR}/ref_src/knowledge_sources"
GRAPH_SRC_DIR="${ROOT_DIR}/src"

clone_or_update() {
  local repo_url="$1"
  local target_dir="$2"

  if [ -d "${target_dir}/.git" ]; then
    git -C "${target_dir}" fetch --depth 1 origin
    git -C "${target_dir}" pull --ff-only --depth 1 origin "$(git -C "${target_dir}" rev-parse --abbrev-ref HEAD)"
    return
  fi

  git clone --depth 1 "${repo_url}" "${target_dir}"
}

mkdir -p "${REF_SRC_DIR}"
mkdir -p "${GRAPH_SRC_DIR}"

# Runtime/testing uses the official SocialMinds Jazzy Debian packages.
# These clones are reference-only and live outside the active src/ workspace.
clone_or_update "https://github.com/pal-robotics/kb_msgs.git" "${REF_SRC_DIR}/kb_msgs"
clone_or_update "https://gitlab.iiia.csic.es/socialminds/neurosymbolic-ai/knowledge_core.git" "${REF_SRC_DIR}/knowledge_core"
clone_or_update "https://gitlab.iiia.csic.es/socialminds/ros4hri/interaction_sim.git" "${REF_SRC_DIR}/interaction_sim"
clone_or_update "https://github.com/severin-lemaignan/openrobots-ontology.git" "${REF_SRC_DIR}/oro"

# These lightweight ROS4HRI workspace packages are useful to keep locally under
# src/ for graph coverage even when runtime normally relies on Debian packages.
clone_or_update "https://github.com/ros4hri/interaction_skills.git" "${GRAPH_SRC_DIR}/interaction_skills"
clone_or_update "https://github.com/ros4hri/std_skills.git" "${GRAPH_SRC_DIR}/std_skills"
clone_or_update "https://github.com/ros4hri/motions_skills.git" "${GRAPH_SRC_DIR}/motions_skills"

# Optional graph-only sources for detector/demo packages that are not publicly
# documented in this repo. If you know their remotes, pass them as env vars.
if [ -n "${EMOROBCARE_CV_MSGS_REPO:-}" ]; then
  clone_or_update "${EMOROBCARE_CV_MSGS_REPO}" "${GRAPH_SRC_DIR}/emorobcare_cv_msgs"
fi

if [ -n "${EMOROBCARE_CV_OBJECT_DETECTION_REPO:-}" ]; then
  clone_or_update \
    "${EMOROBCARE_CV_OBJECT_DETECTION_REPO}" \
    "${GRAPH_SRC_DIR}/emorobcare_cv_object_detection"
fi

if [ -n "${MY_GAME_INTERFACE_REPO:-}" ]; then
  clone_or_update "${MY_GAME_INTERFACE_REPO}" "${GRAPH_SRC_DIR}/my_game_interface"
fi
