#!/usr/bin/env bash
set -euo pipefail

# Simple builder/installer for the HAL component in this directory.
# Usage:
#   ./build_and_install.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if ! command -v ninja >/dev/null 2>&1; then
  echo "Error: ninja is required but not found. Please install ninja-build."
  exit 1
fi
BUILD_DIR="${SCRIPT_DIR}/cmake-build-release"

# Configure
cmake -S "${SCRIPT_DIR}" -B "${BUILD_DIR}" -G Ninja \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/usr

# Build
cmake --build "${BUILD_DIR}" -j"$(nproc)"

# Install
sudo cmake --install "${BUILD_DIR}"
sudo halcompile --install "${SCRIPT_DIR}/level_map.comp"
sudo halcompile --install "${SCRIPT_DIR}/probe_helper.comp"
