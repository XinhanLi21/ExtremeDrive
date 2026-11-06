#!/usr/bin/env bash
set -euo pipefail
extremedrive --assets "${ASSETS:-assets}" download --scope "${SCOPE:-execution}" --dataroot "${DATAROOT:-data}" "$@"
