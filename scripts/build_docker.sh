#!/bin/bash
set -e

echo "Building NAO-ROS4HRI Docker image..."
docker build -t nao-ros4hri:jazzy -f docker/Dockerfile .

echo "Build complete! Run with:"
echo "  docker run -it --rm --net=host nao-ros4hri:jazzy"
