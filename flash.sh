#!/usr/bin/env bash

set -e

./linuxcnc/pi_init/08-firmware-user.sh

sudo ninja -C cmake-build-release flash-electrophorus
