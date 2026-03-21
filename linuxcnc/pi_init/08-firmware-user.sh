#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MACHINES_DIR="$SCRIPT_DIR/../hal_comps/machines"
AVAILABLE=$(ls -1 "$MACHINES_DIR")

MACHINE="${1:-}"
if [ -z "$MACHINE" ] || [ ! -d "$MACHINES_DIR/$MACHINE" ]; then
  echo "Usage: $0 <machine>"
  echo -e "Available machines:\n$AVAILABLE"
  exit 1
fi

git submodule init
git submodule update --depth 1

mkdir -p cmake-build-release
cmake -Bcmake-build-release -GNinja -DCMAKE_BUILD_TYPE=Release -DMBED_TARGET=LPC1768 -DMACHINE="$MACHINE"
ninja -C cmake-build-release