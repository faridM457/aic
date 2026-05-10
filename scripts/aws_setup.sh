#!/bin/bash
# scripts/aws_setup.sh
# Run ONCE on a fresh Ubuntu 24.04 EC2 g5.2xlarge (NVIDIA A10G, 24 GB VRAM).
# Fully non-interactive and idempotent.
set -euo pipefail

echo "========================================================"
echo "AIC AWS Setup — $(date)"
echo "========================================================"

# ---------------------------------------------------------------
# 1. System dependencies
# ---------------------------------------------------------------
echo "[1/8] System deps..."
sudo apt-get update -y
sudo DEBIAN_FRONTEND=noninteractive apt-get upgrade -y
sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
  tmux curl wget git ca-certificates gnupg lsb-release \
  distrobox xvfb x11-xserver-utils xdotool

# ---------------------------------------------------------------
# 2. Docker (official apt method — NOT snap)
# ---------------------------------------------------------------
echo "[2/8] Docker..."
if ! command -v docker &>/dev/null; then
  sudo install -m 0755 -d /etc/apt/keyrings
  curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
    | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
  sudo chmod a+r /etc/apt/keyrings/docker.gpg
  echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] \
    https://download.docker.com/linux/ubuntu \
    $(lsb_release -cs) stable" \
    | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
  sudo apt-get update -y
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y \
    docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
fi

# Add current user to docker group (takes effect after re-login / newgrp docker)
sudo usermod -aG docker "$USER" || true
echo "Docker version: $(docker --version 2>/dev/null || echo 'installed – re-login needed')"

# ---------------------------------------------------------------
# 3. NVIDIA Container Toolkit
# ---------------------------------------------------------------
echo "[3/8] NVIDIA Container Toolkit..."
if ! dpkg -l | grep -q nvidia-container-toolkit; then
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
    | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
    | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
    | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
  sudo apt-get update -y
  sudo DEBIAN_FRONTEND=noninteractive apt-get install -y nvidia-container-toolkit
fi
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
echo "NVIDIA Container Toolkit installed."

# Add user to render group for GPU hardware access in distrobox
sudo usermod -a -G render "$USER"
echo "NOTE: render group takes effect on next login (newgrp render or re-SSH)"

# ---------------------------------------------------------------
# 4. Pixi
# ---------------------------------------------------------------
echo "[4/8] Pixi..."
if ! command -v pixi &>/dev/null; then
  curl -fsSL https://pixi.sh/install.sh | sh
fi
grep -qF 'export PATH="$HOME/.pixi/bin:$PATH"' ~/.bashrc \
  || echo 'export PATH="$HOME/.pixi/bin:$PATH"' >> ~/.bashrc
export PATH="$HOME/.pixi/bin:$PATH"
echo "Pixi version: $(pixi --version)"

# ---------------------------------------------------------------
# 5. Virtual display for headless Gazebo
# ---------------------------------------------------------------
echo "[5/8] Virtual display (Xvfb)..."
grep -qF 'export DISPLAY=:99' ~/.bashrc \
  || echo 'export DISPLAY=:99' >> ~/.bashrc
grep -qF 'Xvfb :99' ~/.bashrc \
  || echo 'pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &' >> ~/.bashrc
export DISPLAY=:99
pgrep Xvfb >/dev/null || Xvfb :99 -screen 0 1280x1024x24 &
sleep 2
echo "Virtual display running on :99"

# ---------------------------------------------------------------
# 6. Clone the AIC repo and install pixi deps
# ---------------------------------------------------------------
echo "[6/8] Cloning repo and installing deps..."
mkdir -p ~/ws_aic/src
if [ ! -d ~/ws_aic/src/aic/.git ]; then
  cd ~/ws_aic/src
  git clone https://github.com/faridM457/aic
fi
cd ~/ws_aic/src/aic
git pull origin main
pixi install --locked
echo "Pixi install complete."

# ---------------------------------------------------------------
# 7. Pull eval container and create distrobox
# ---------------------------------------------------------------
echo "[7/8] Pulling eval container and creating distrobox..."
export DBX_CONTAINER_MANAGER=docker

# sg docker runs the commands in the docker group without needing re-login
sg docker -c "docker pull ghcr.io/intrinsic-dev/aic/aic_eval:latest"

# Create distrobox only if it doesn't exist yet
sg docker -c "distrobox list 2>/dev/null | grep -q aic_eval \
  || distrobox create -r --nvidia \
       -i ghcr.io/intrinsic-dev/aic/aic_eval:latest \
       aic_eval"
echo "Distrobox aic_eval ready."

# ---------------------------------------------------------------
# 8. Verification — print PASS/FAIL for each check
# ---------------------------------------------------------------
echo "[8/8] Verification..."
echo ""

pass_fail() {
  local label="$1"; shift
  if "$@" &>/dev/null; then
    echo "  PASS  $label"
  else
    echo "  FAIL  $label"
  fi
}

pass_fail "nvidia-smi" nvidia-smi
pass_fail "docker GPU passthrough" \
  sg docker -c "docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi"
pass_fail "distrobox --version" distrobox --version
pass_fail "pixi --version" pixi --version
pass_fail "python3 --version" python3 --version

echo ""
echo "========================================================"
echo "Setup complete."
echo ""
echo "NEXT STEPS:"
echo "  1. Run:  newgrp docker && newgrp render"
echo "     (or log out and back in to pick up the docker + render groups)"
echo "  2. Run:  scripts/verify_env.sh"
echo "========================================================"
