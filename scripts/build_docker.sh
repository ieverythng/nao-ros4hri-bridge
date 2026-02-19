#!/bin/bash
set -e

PROFILE="${1:-overlay}"
TAG="${2:-iiia:nao}"
BASE_IMAGE="${BASE_IMAGE:-iiia:nao}"

if [ "${PROFILE}" = "overlay" ]; then
  echo "Building overlay image '${TAG}' from base '${BASE_IMAGE}'..."
  docker build \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -t "${TAG}" \
    -f docker/Dockerfile .
elif [ "${PROFILE}" = "full" ]; then
  echo "Building full image '${TAG}' from docker/Dockerfile.full..."
  docker build -t "${TAG}" -f docker/Dockerfile.full .
else
  echo "Unknown profile '${PROFILE}'. Use 'overlay' or 'full'."
  exit 1
fi

echo "Build complete! Run with:"
echo "  docker run -it --rm --net=host ${TAG}"
