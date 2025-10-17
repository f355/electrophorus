#!/usr/bin/env bash

set -e

apt install -y linuxcnc-uspace linuxcnc-uspace-dev python3-qtpyvcp python3-probe-basic flexgui

# linuxcnc 2.9.4 is checking /sys/kernel/realtime, RPi kernel doesn't have that, so we need to use force
echo "export LINUXCNC_FORCE_REALTIME=1" >/etc/profile.d/linuxcnc-force-realtime.sh