#!/bin/bash
# scripts/check_gpu.sh
# Run while Gazebo is active to verify GPU passthrough is working.
set -euo pipefail

echo "=== GPU Status ==="
nvidia-smi

echo ""
echo "=== GPU Processes ==="
nvidia-smi --query-compute-apps=pid,name,used_memory --format=csv

echo ""
echo "=== Render Group ==="
groups | grep render && echo "PASS: in render group" || echo "FAIL: not in render group (run: sudo usermod -a -G render \$USER && newgrp render)"

echo ""
echo "=== Docker GPU Test ==="
docker run --rm --gpus all nvidia/cuda:12.0-base nvidia-smi | head -5
