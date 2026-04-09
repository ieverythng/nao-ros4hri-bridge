#!/usr/bin/env bash

set -euo pipefail

BOOTSTRAP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KNOWLEDGE_DIR="$(cd "${BOOTSTRAP_DIR}/.." && pwd)"

TARGET_REPO="${1:-}"
if [[ -z "${TARGET_REPO}" ]]; then
  printf 'Usage: %s <target-repo-path>\n' "${0}" >&2
  exit 1
fi

TARGET_REPO="$(cd "${TARGET_REPO}" && pwd)"
TARGET_NAME="$(basename "${TARGET_REPO}")"
TARGET_TOOLS="${TARGET_REPO}/tools/knowledge"
TARGET_DOCS="${TARGET_REPO}/docs/knowledge"
TEMPLATE_DIR="${BOOTSTRAP_DIR}/templates"
RUNTIME_FILES=(
  gitnexus.sh
  lib.sh
  setup_gitnexus.sh
  index_repo.sh
  generate_wiki.sh
  serve_http_mcp.sh
  status.sh
  status_summary.py
  pre_commit_advisory.sh
  post_commit_refresh.sh
  codex_with_gitnexus.sh
  sync_artifacts.py
)

mkdir -p "${TARGET_TOOLS}" "${TARGET_DOCS}/wiki" "${TARGET_REPO}/.codex" "${TARGET_REPO}/.cursor"

write_template_if_missing() {
  local source_file="$1"
  local target_file="$2"

  if [[ -e "${target_file}" ]]; then
    printf 'Skipped existing file: %s\n' "${target_file}"
    return
  fi

  sed "s/__REPO_NAME__/${TARGET_NAME}/g" "${source_file}" > "${target_file}"
}

for file in "${RUNTIME_FILES[@]}"; do
  cp "${KNOWLEDGE_DIR}/${file}" "${TARGET_TOOLS}/${file}"
done

for file in "${RUNTIME_FILES[@]}"; do
  case "${file}" in
    *.sh|*.py)
      chmod +x "${TARGET_TOOLS}/${file}"
      ;;
  esac
done

cp -R "${BOOTSTRAP_DIR}" "${TARGET_TOOLS}/"

write_template_if_missing "${TEMPLATE_DIR}/AGENTS.md.tpl" "${TARGET_REPO}/AGENTS.md"
write_template_if_missing "${TEMPLATE_DIR}/codex-config.toml.tpl" "${TARGET_REPO}/.codex/config.toml"
write_template_if_missing "${TEMPLATE_DIR}/cursor-mcp.json.tpl" "${TARGET_REPO}/.cursor/mcp.json"
write_template_if_missing "${TEMPLATE_DIR}/docs-knowledge-README.md.tpl" "${TARGET_DOCS}/README.md"
write_template_if_missing "${TEMPLATE_DIR}/docs-knowledge-QUICKSTART.md.tpl" "${TARGET_DOCS}/QUICKSTART.md"
write_template_if_missing "${TEMPLATE_DIR}/docs-knowledge-WORKFLOWS.md.tpl" "${TARGET_DOCS}/WORKFLOWS.md"
write_template_if_missing "${TEMPLATE_DIR}/docs-knowledge-EVAL.md.tpl" "${TARGET_DOCS}/EVAL.md"
write_template_if_missing "${TEMPLATE_DIR}/docs-knowledge-DECISIONS.md.tpl" "${TARGET_DOCS}/DECISIONS.md"
write_template_if_missing "${TEMPLATE_DIR}/wiki-README.md.tpl" "${TARGET_DOCS}/wiki/README.md"
write_template_if_missing "${TEMPLATE_DIR}/module_tree.json.tpl" "${TARGET_DOCS}/module_tree.json"
write_template_if_missing "${TEMPLATE_DIR}/INDEX_STATUS.md.tpl" "${TARGET_DOCS}/INDEX_STATUS.md"

printf '\nGitNexus knowledge bootstrap copied into %s\n' "${TARGET_REPO}"
printf 'Next steps:\n'
printf '  1. %s/tools/knowledge/setup_gitnexus.sh\n' "${TARGET_REPO}"
printf '  2. %s/tools/knowledge/index_repo.sh\n' "${TARGET_REPO}"
