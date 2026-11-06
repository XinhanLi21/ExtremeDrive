#!/usr/bin/env bash
set -euo pipefail
# Run from the repository root; paths are supplied explicitly.
: "${SUBMISSION:?Set SUBMISSION to the 131-window prediction JSONL}"
extremedrive --assets "${ASSETS:-assets}" evaluate --submission "$SUBMISSION" --workers "${WORKERS:-4}" --out "${OUTPUT_DIR:-results/evaluation}"
