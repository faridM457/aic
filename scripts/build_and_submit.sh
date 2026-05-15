#!/bin/bash
# scripts/build_and_submit.sh
# Run LOCALLY after copy_checkpoints.sh to build and push submission image.
#
# Usage:  ./scripts/build_and_submit.sh <version_tag>
# Example: ./scripts/build_and_submit.sh v3
#
# Requires:
#   $AIC_TEAM_NAME  — your ECR team name (e.g. "team-alpha")
#   AWS CLI configured with ECR push permissions
#   Docker daemon running
#
# ECR tags are IMMUTABLE. Increment the version tag on every push.
# You cannot overwrite a tag once pushed.
set -euo pipefail

VERSION=${1:?Usage: $0 <version_tag>  (e.g. v1, v2, v3)}
TEAM="${AIC_TEAM_NAME:?Set AIC_TEAM_NAME env var to your ECR team name}"

ECR_REGISTRY="973918476471.dkr.ecr.us-east-1.amazonaws.com"
ECR_REPO="aic-team/${TEAM}"
ECR_URI="${ECR_REGISTRY}/${ECR_REPO}"
FULL_TAG="${ECR_URI}:${VERSION}"

AIC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$AIC_DIR"

echo "========================================================"
echo "AIC build & submit — version ${VERSION}"
echo "Team:    ${TEAM}"
echo "ECR URI: ${FULL_TAG}"
echo "========================================================"
echo ""

# ---------------------------------------------------------------
# 1. Stage checkpoints into Docker build context
# ---------------------------------------------------------------
echo "[1/6] Staging checkpoints..."

SFP_SRC=""
SC_SRC=""

for TRIAL in 1 2; do
  DIR="outputs/act_trial${TRIAL}/checkpoints/last/pretrained_model"
  if [ -d "$DIR" ]; then
    SFP_SRC="$DIR"
    echo "  SFP model: $SFP_SRC (trial ${TRIAL})"
    break
  fi
done

if [ -d "outputs/act_trial3/checkpoints/last/pretrained_model" ]; then
  SC_SRC="outputs/act_trial3/checkpoints/last/pretrained_model"
  echo "  SC  model: $SC_SRC (trial 3)"
fi

if [ -z "$SFP_SRC" ]; then
  echo "ERROR: No SFP checkpoint found."
  echo "  Expected: outputs/act_trial1/checkpoints/last/pretrained_model/ or outputs/act_trial2/checkpoints/last/pretrained_model/"
  echo "  Run: ./scripts/copy_checkpoints.sh <ec2-dns>  first."
  exit 1
fi

mkdir -p aic_example_policies/checkpoints/sfp

cp -r "${SFP_SRC}/." aic_example_policies/checkpoints/sfp/
echo "  Staged SFP → aic_example_policies/checkpoints/sfp/"

if [ -n "$SC_SRC" ]; then
  mkdir -p aic_example_policies/checkpoints/sc
  cp -r "${SC_SRC}/." aic_example_policies/checkpoints/sc/
  echo "  Staged SC  → aic_example_policies/checkpoints/sc/"
else
  echo "  WARNING: Trial 3 (SC) checkpoint not found — SFP model will be used as fallback for SC."
  mkdir -p aic_example_policies/checkpoints/sc
  cp -r "${SFP_SRC}/." aic_example_policies/checkpoints/sc/
fi

# Verify Dockerfile points to per-connector checkpoint dirs.
DOCKERFILE="docker/my_solution/Dockerfile"
if ! grep -q "ACT_MODEL_PATH_SFP=/ws_aic/src/aic/aic_example_policies/checkpoints/sfp" "$DOCKERFILE"; then
  echo "ERROR: $DOCKERFILE is missing ACT_MODEL_PATH_SFP."
  exit 1
fi
if ! grep -q "ACT_MODEL_PATH_SC=/ws_aic/src/aic/aic_example_policies/checkpoints/sc" "$DOCKERFILE"; then
  echo "ERROR: $DOCKERFILE is missing ACT_MODEL_PATH_SC."
  exit 1
fi
echo "  Dockerfile ENV verified for per-connector model paths."

# Verify checkpoint files are present
for SUBDIR in sfp sc; do
  COUNT=$(ls "aic_example_policies/checkpoints/${SUBDIR}/" 2>/dev/null | wc -l)
  if [ "$COUNT" -eq 0 ]; then
    echo "ERROR: aic_example_policies/checkpoints/${SUBDIR}/ is empty after staging."
    exit 1
  fi
  if [ ! -f "aic_example_policies/checkpoints/${SUBDIR}/config.json" ] || \
     [ ! -f "aic_example_policies/checkpoints/${SUBDIR}/model.safetensors" ]; then
    echo "ERROR: aic_example_policies/checkpoints/${SUBDIR}/ is not a LeRobot pretrained_model directory."
    echo "Expected config.json and model.safetensors at the top level."
    exit 1
  fi
  echo "  checkpoints/${SUBDIR}/: ${COUNT} files"
done

# ---------------------------------------------------------------
# 2. Build image
# ---------------------------------------------------------------
echo ""
echo "[2/6] Building Docker image (my-solution:v1)..."
docker compose -f docker/docker-compose.yaml build model
echo "  Build complete."

# ---------------------------------------------------------------
# 3. Optional local smoke test
# ---------------------------------------------------------------
echo ""
echo "[3/6] Local smoke test (verify container starts and imports policy)..."
if docker run --rm --entrypoint python3 my-solution:v1 \
     -c "import aic_example_policies.ros.CableInsertion; print('OK')" 2>&1 | grep -q "OK"; then
  echo "  Smoke test PASSED."
else
  echo "  WARNING: Smoke test failed — check image before pushing."
  echo "  Run manually: docker run --rm --entrypoint python3 my-solution:v1 -c \"import aic_example_policies.ros.CableInsertion; print('OK')\""
  read -rp "  Continue anyway? [y/N] " CONT
  [[ "$CONT" =~ ^[Yy]$ ]] || exit 1
fi

# ---------------------------------------------------------------
# 4. ECR authentication
# ---------------------------------------------------------------
echo ""
echo "[4/6] Authenticating to ECR..."
aws ecr get-login-password --region us-east-1 \
  | docker login --username AWS --password-stdin "$ECR_REGISTRY"
echo "  ECR auth OK."

# ---------------------------------------------------------------
# 5. Tag image
# ---------------------------------------------------------------
echo ""
echo "[5/6] Tagging image..."
docker tag my-solution:v1 "${FULL_TAG}"
echo "  Tagged: ${FULL_TAG}"

# Immutable tag check — ECR will reject a duplicate, but warn early.
echo ""
echo "  ⚠  REMINDER: ECR tags are IMMUTABLE."
echo "     If tag '${VERSION}' already exists for repo '${ECR_REPO}', the push WILL FAIL."
echo "     Increment the version (e.g. v$((${VERSION#v}+1))) and re-run."
echo ""
read -rp "  Confirm push of '${FULL_TAG}'? [y/N] " PUSH_OK
[[ "$PUSH_OK" =~ ^[Yy]$ ]] || { echo "Aborted."; exit 0; }

# ---------------------------------------------------------------
# 6. Push to ECR
# ---------------------------------------------------------------
echo ""
echo "[6/6] Pushing to ECR..."
docker push "${FULL_TAG}"
echo ""
echo "========================================================"
echo "PUSH SUCCESSFUL"
echo ""
echo "Full image URI:"
echo "  ${FULL_TAG}"
echo ""
echo "PORTAL SUBMISSION STEPS:"
echo "  1. Go to the AIC participant portal."
echo "  2. Navigate to 'Submissions' or 'Submit Model'."
echo "  3. In the image URI field, paste exactly:"
echo ""
echo "     ${FULL_TAG}"
echo ""
echo "  4. Select the evaluation config (Qualification Round)."
echo "  5. Click Submit and monitor the evaluation dashboard."
echo ""
echo "  ⚠  If you need to re-submit, you MUST use a new tag:"
echo "     ./scripts/build_and_submit.sh v$((${VERSION#v}+1))"
echo "========================================================"
