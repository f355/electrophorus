#!/usr/bin/env bash

set -e

sudo ./linuxcnc/pi_init/01-apt.sh
sudo ./linuxcnc/pi_init/02-kernel.sh
sudo USERNAME="$USER" ./linuxcnc/pi_init/03-xfce.sh
sudo USERNAME="$USER" ./linuxcnc/pi_init/04-vnc.sh
sudo ./linuxcnc/pi_init/05-openocd.sh
sudo ./linuxcnc/pi_init/06-linuxcnc.sh
./linuxcnc/pi_init/07-linuxcnc-user.sh
./linuxcnc/pi_init/08-firmware-user.sh
sudo USERNAME="$USER" ./linuxcnc/pi_init/09-samba.sh


echo "Please reboot to apply changes."