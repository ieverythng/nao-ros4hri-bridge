#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC_DIR="${ROOT_DIR}/src"

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

clone_or_update "https://github.com/pal-robotics/kb_msgs.git" "${SRC_DIR}/kb_msgs"
clone_or_update "https://gitlab.iiia.csic.es/socialminds/neurosymbolic-ai/knowledge_core.git" "${SRC_DIR}/knowledge_core"
clone_or_update "https://gitlab.iiia.csic.es/socialminds/ros4hri/interaction_sim.git" "${SRC_DIR}/interaction_sim"
