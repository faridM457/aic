#!/bin/bash
# scripts/copy_checkpoints.sh
# Run LOCALLY after training to pull all three checkpoints from EC2.
#
# Usage:  ./scripts/copy_checkpoints.sh <ec2-public-dns>
# Example: ./scripts/copy_checkpoints.sh ec2-12-34-56-78.compute-1.amazonaws.com
#
# Requires:
#   $AIC_KEY_PATH  — path to your EC2 PEM key (default: ~/aic-key.pem)
set -euo pipefail

EC2_HOST=${1:?Usage: $0 <ec2-public-dns>}
KEY="${AIC_KEY_PATH:-$HOME/aic-key.pem}"

if [ ! -f "$KEY" ]; then
  echo "ERROR: EC2 key not found at $KEY"
  echo "Set AIC_KEY_PATH env var or place key at ~/aic-key.pem"
  exit 1
fi

SCP="scp -i $KEY -o StrictHostKeyChecking=no"

echo "Downloading checkpoints from $EC2_HOST ..."

for TRIAL in 1 2 3; do
  LOCAL_DIR="outputs/act_trial${TRIAL}/checkpoints/last/pretrained_model"
  REMOTE_DIR="~/ws_aic/src/aic/outputs/act_trial${TRIAL}/checkpoints/last/pretrained_model"

  mkdir -p "$LOCAL_DIR"

  echo "  Trial ${TRIAL}: ${REMOTE_DIR}/ → ${LOCAL_DIR}/"
  $SCP -r "ubuntu@${EC2_HOST}:${REMOTE_DIR}/." "${LOCAL_DIR}/"

  if [ ! -f "${LOCAL_DIR}/config.json" ] || [ ! -f "${LOCAL_DIR}/model.safetensors" ]; then
    echo "ERROR: Trial ${TRIAL} checkpoint not downloaded. Check EC2 training status."
    echo "Expected LeRobot 0.5.1 checkpoint files under: ${LOCAL_DIR}/"
    exit 1
  fi
  echo "  Trial ${TRIAL}: OK ($(ls "${LOCAL_DIR}/" | wc -l) files)"
done

echo ""
echo "All checkpoints downloaded:"
echo "  outputs/act_trial1/checkpoints/last/pretrained_model/"
echo "  outputs/act_trial2/checkpoints/last/pretrained_model/"
echo "  outputs/act_trial3/checkpoints/last/pretrained_model/"
echo ""
echo "Next: ./scripts/build_and_submit.sh"
