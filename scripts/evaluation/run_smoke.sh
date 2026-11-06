#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")/../.."
export OPENBLAS_NUM_THREADS="${OPENBLAS_NUM_THREADS:-1}"
export OMP_NUM_THREADS="${OMP_NUM_THREADS:-1}"
extremedrive --assets assets evaluate --submission examples/smoke_predictions.jsonl --subset --workers "${WORKERS:-3}" --out "${OUTPUT_DIR:-results/smoke}"
