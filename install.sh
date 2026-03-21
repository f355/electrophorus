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

sudo ./linuxcnc/pi_init/01-apt.sh
sudo ./linuxcnc/pi_init/02-kernel.sh
sudo USERNAME="$USER" ./linuxcnc/pi_init/03-xfce.sh
sudo USERNAME="$USER" ./linuxcnc/pi_init/04-vnc.sh
sudo ./linuxcnc/pi_init/05-openocd.sh
sudo ./linuxcnc/pi_init/06-linuxcnc.sh
./linuxcnc/pi_init/07-linuxcnc-user.sh "$MACHINE"
./linuxcnc/pi_init/08-firmware-user.sh "$MACHINE"
sudo USERNAME="$USER" ./linuxcnc/pi_init/09-samba.sh

echo "Please reboot to apply changes."