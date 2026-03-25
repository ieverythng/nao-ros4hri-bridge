#!/bin/bash
set -e

PROFILE="${1:-overlay}"
TAG="${2:-iiia:nao-overlay}"
BASE_IMAGE="${BASE_IMAGE:-iiia:nao}"

build_cmd=(docker build)
if docker buildx version >/dev/null 2>&1; then
  build_cmd=(env DOCKER_BUILDKIT=1 docker build --progress=plain)
else
  echo "docker buildx is unavailable; falling back to classic docker build output."
fi

if [ "${PROFILE}" = "overlay" ] && [ "${TAG}" = "${BASE_IMAGE}" ]; then
  echo "Refusing overlay build: TAG ('${TAG}') matches BASE_IMAGE ('${BASE_IMAGE}')."
  echo "This creates recursive image layering and can fail with 'max depth exceeded'."
  echo "Use different values, for example:"
  echo "  BASE_IMAGE=iiia:nao-working-legacy ./scripts/build_docker.sh overlay iiia:nao"
  exit 2
fi

if [ "${PROFILE}" = "overlay" ]; then
  echo "Building overlay image '${TAG}' from base '${BASE_IMAGE}'..."
  "${build_cmd[@]}" \
    --build-arg BASE_IMAGE="${BASE_IMAGE}" \
    -t "${TAG}" \
    -f docker/Dockerfile .
elif [ "${PROFILE}" = "full" ]; then
  echo "Building full image '${TAG}' from docker/Dockerfile.full..."
  "${build_cmd[@]}" -t "${TAG}" -f docker/Dockerfile.full .
else
  echo "Unknown profile '${PROFILE}'. Use 'overlay' or 'full'."
  exit 1
fi

echo "Build complete! Run with:"
echo "  docker run -it --rm --net=host ${TAG}"
