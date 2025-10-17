#!/usr/bin/env bash

set -e

./linuxcnc/pi_init/07-linuxcnc-user.sh

sudo ninja -C cmake-build-release flash-electrophorus
