#!/usr/bin/env bash

set -euo pipefail

echo "[Docker Install] Starting Docker setup for Ubuntu (tested on 22.04)…"

# Ensure running on Ubuntu
if ! grep -qi ubuntu /etc/os-release; then
  echo "[Docker Install] This script is intended for Ubuntu. Aborting." >&2
  exit 1
fi

# Require sudo privileges early
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "[Docker Install] sudo not found. Please run as root." >&2
    exit 1
  fi
fi

SUDO=""
if [ "${EUID:-$(id -u)}" -ne 0 ]; then
  SUDO="sudo"
fi

echo "[Docker Install] Updating apt and installing prerequisites…"
$SUDO apt-get update -y
$SUDO apt-get install -y ca-certificates curl gnupg lsb-release apt-transport-https

echo "[Docker Install] Setting up Docker APT repository…"
$SUDO install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | $SUDO gpg --dearmor -o /etc/apt/keyrings/docker.gpg
$SUDO chmod a+r /etc/apt/keyrings/docker.gpg

ARCH=$(dpkg --print-architecture)
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
echo "deb [arch=${ARCH} signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu ${CODENAME} stable" | $SUDO tee /etc/apt/sources.list.d/docker.list >/dev/null

echo "[Docker Install] Installing Docker Engine and plugins…"
$SUDO apt-get update -y
$SUDO apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

echo "[Docker Install] Enabling and starting Docker service…"
$SUDO systemctl enable docker
$SUDO systemctl start docker

CURRENT_USER=${SUDO_USER:-$USER}
echo "[Docker Install] Adding user '${CURRENT_USER}' to 'docker' group (no sudo needed for docker)…"
$SUDO usermod -aG docker "$CURRENT_USER" || true

echo "[Docker Install] Verifying Docker with hello-world (may require new shell for group change)…"
if command -v docker >/dev/null 2>&1; then
  # Try with sudo first (in case group change not active yet)
  if ! docker run --rm hello-world >/dev/null 2>&1; then
    $SUDO docker run --rm hello-world || true
  fi
fi

echo "[Docker Install] Pulling Micro-ROS Agent image (optional)…"
if command -v docker >/dev/null 2>&1; then
  docker pull microros/micro-ros-agent:humble || $SUDO docker pull microros/micro-ros-agent:humble || true
fi

cat <<EOF

[Docker Install] Success!

Next steps:
1) Open a NEW terminal (or run: newgrp docker) so your docker group membership takes effect.
2) Test non-root Docker:
   docker run --rm hello-world
3) Start Micro-ROS Agent when needed:
   docker run -it --rm -v /dev:/dev -v /dev/shm:/dev/shm \
     --privileged --net=host microros/micro-ros-agent:humble udp4 --port 8090 -v4

EOF


