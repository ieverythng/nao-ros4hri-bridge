#!/bin/bash
set -euo pipefail

ARCHIVE_PATH="${1:-docker/iiia_nao_image.tar.gz}"

if [ ! -f "${ARCHIVE_PATH}" ]; then
  echo "Archive not found: ${ARCHIVE_PATH}"
  exit 1
fi

echo "Importing docker image from '${ARCHIVE_PATH}'..."
gunzip -c "${ARCHIVE_PATH}" | docker load
echo "Done."
