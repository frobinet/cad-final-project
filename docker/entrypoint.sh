#!/bin/bash
set -e

# Docker host uid/gid should be available as env variables
HOST_UID=${DOCKER_HOST_UID:-}
HOST_GID=${DOCKER_HOST_GID:-}

if [ "$(id -u)" != "0" ]; then
    >&2 echo "ERROR: Not running as root. Please run as root, and set DOCKER_HOST_UID and DOCKER_HOST_GID environment variables."
    exit 120
elif [ "${HOST_UID}" = "" ]; then
    >&2 echo "ERROR: HOST_UID is not set. Please run as root, and set DOCKER_HOST_UID and DOCKER_HOST_GID environment variables."
    exit 121
elif [ "${HOST_GID}" = "" ]; then
    >&2 echo "ERROR: HOST_GID is not set. Please run as root, and set DOCKER_HOST_UID and DOCKER_HOST_GID environment variables."
    exit 122
fi

# if id -u "$HOST_UID" >/dev/null 2>&1; then
#     echo "User 'user' exists, not recreating"
# else
#     # (Re)create the group 'user'
#     addgroup --gid ${HOST_GID} user
#     # (Re)create the user 'user'
#     useradd user -u ${HOST_UID} -g ${HOST_GID} -m -s /bin/bash
# fi
usermod -u ${HOST_UID} user
groupmod -g ${HOST_GID} user

echo "Running server with UID=${HOST_UID} GID=${HOST_GID}"
echo "Command: $@"
exec runuser -u user -- "$@"
