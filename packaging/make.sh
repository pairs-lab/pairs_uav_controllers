#!/usr/bin/env bash
#
# Host entrypoint: builds the ros:noetic packaging image and runs it to produce
# ros-noetic-pairs-uav-controllers_*.deb and its PAIRS dependency .debs.
#
# Output .deb files land in   <repo>/packaging/output/
# Requirements on the host: docker.
#
# Depends on pairs_lib + pairs_msgs + pairs_uav_hw_api + pairs_uav_managers — all
# must be checked out next to this repo in the same src/ directory, each on its
# `ros1` branch.
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
SRC_DIR="$(dirname "$REPO_DIR")"
OUTPUT_DIR="$SCRIPT_DIR/output"
IMAGE="pairs-controllers-noetic-packaging"

mkdir -p "$OUTPUT_DIR"

echo ">> building packaging image ($IMAGE)..."
docker build -t "$IMAGE" "$SCRIPT_DIR"

echo ">> building .deb packages (output -> $OUTPUT_DIR)..."
docker run --rm \
  -v "$SRC_DIR":/src:ro \
  -v "$OUTPUT_DIR":/output \
  "$IMAGE"

echo ">> done. Artifacts:"
ls -1 "$OUTPUT_DIR"/*.deb
