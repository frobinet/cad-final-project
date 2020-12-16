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
    360lab/cad-final-project bash

echo "Container '$NAME' is now running. You can develop inside it by attaching VSCode to it."

