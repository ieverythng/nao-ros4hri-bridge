#!/usr/bin/env bash

set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REF_SRC_DIR="${ROOT_DIR}/ref_src/knowledge_sources"

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

# Runtime/testing uses the official SocialMinds Jazzy Debian packages.
# These clones are reference-only and live outside the active src/ workspace.
clone_or_update "https://github.com/pal-robotics/kb_msgs.git" "${REF_SRC_DIR}/kb_msgs"
clone_or_update "https://gitlab.iiia.csic.es/socialminds/neurosymbolic-ai/knowledge_core.git" "${REF_SRC_DIR}/knowledge_core"
clone_or_update "https://gitlab.iiia.csic.es/socialminds/ros4hri/interaction_sim.git" "${REF_SRC_DIR}/interaction_sim"
clone_or_update "https://github.com/severin-lemaignan/openrobots-ontology.git" "${REF_SRC_DIR}/oro"
