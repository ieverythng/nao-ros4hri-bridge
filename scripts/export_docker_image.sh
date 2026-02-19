#!/bin/bash
set -euo pipefail

IMAGE_TAG="${1:-iiia:nao}"
OUTPUT_PATH="${2:-docker/iiia_nao_image.tar.gz}"

echo "Exporting image '${IMAGE_TAG}' to '${OUTPUT_PATH}'..."
docker save "${IMAGE_TAG}" | gzip > "${OUTPUT_PATH}"
echo "Done."
