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

mkdir -p ~/linuxcnc/configs
mkdir -p ~/linuxcnc/nc_files

ln -sf /usr/share/linuxcnc/ncfiles ~/linuxcnc/nc_files/examples
ln -sf /usr/share/linuxcnc/ncfiles/gcmc_lib ~/linuxcnc/nc_files/gcmc_lib
ln -sf /usr/share/linuxcnc/ncfiles/gladevcp_lib ~/linuxcnc/nc_files/gladevcp_lib
ln -sf /usr/share/linuxcnc/ncfiles/ngcgui_lib ~/linuxcnc/nc_files/ngcgui_lib
ln -sf /usr/share/linuxcnc/ncfiles/remap_lib ~/linuxcnc/nc_files/remap_lib

for dir in "$PWD"/linuxcnc/config/*/; do
  ln -sf "$dir" ~/linuxcnc/configs/"$(basename "$dir")"
done

./linuxcnc/hal_comps/build_and_install.sh -DMACHINE="$MACHINE"
