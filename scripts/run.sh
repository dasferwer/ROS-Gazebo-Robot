#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose build
docker compose run --rm --service-ports robot bash scripts/launch.sh "$@"
