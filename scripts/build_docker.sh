#!/bin/bash
set -e

echo "Building NAO-ROS4HRI Docker image..."
docker build -t iiia:nao -f docker/Dockerfile .

echo "Build complete! Run with:"
echo "  docker run -it --rm --net=host iiia:nao"
