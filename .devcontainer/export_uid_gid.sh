#!/bin/bash
echo "DOCKER_HOST_UID=$(id -u)" > .devcontainer/devcontainer.env
echo "DOCKER_HOST_GID=$(id -g)" >> .devcontainer/devcontainer.env