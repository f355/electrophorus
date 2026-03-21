#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MACHINES_DIR="$SCRIPT_DIR/linuxcnc/hal_comps/machines"
AVAILABLE=$(ls -1 "$MACHINES_DIR")

MACHINE="${1:-}"
if [ -z "$MACHINE" ] || [ ! -d "$MACHINES_DIR/$MACHINE" ]; then
  echo "Usage: $0 <machine>"
  echo -e "Available machines:\n$AVAILABLE"
  exit 1
fi

./linuxcnc/pi_init/08-firmware-user.sh "$MACHINE"

sudo ninja -C cmake-build-release flash-electrophorus
