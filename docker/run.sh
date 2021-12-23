#!/bin/bash
set -e

# TODO Provide support for GPU
NAME="cad-final-project"
docker run -it --rm -d \
    --name "$NAME" \
    -p 4567:4567 \
    -v /tmp/log:/root/.ros \
    -v $(pwd):/host \
    --workdir /host \
    -e DOCKER_HOST_UID=$(id -u) \
    -e DOCKER_HOST_GID=$(id -g) \
    360lab/cad-final-project bash

echo "Container '$NAME' is now running. You can develop inside it by attaching VSCode to it."

