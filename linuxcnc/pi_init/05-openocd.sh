#!/usr/bin/env bash

set -e

apt install -y libtool libjim-dev pkg-config libgpiod-dev

# build openocd from source with linuxgpiod support - debian packages don't have it for reasons unknown
git clone https://github.com/raspberrypi/openocd.git /tmp/openocd
pushd /tmp/openocd
./bootstrap
./configure --prefix=/usr --enable-linuxgpiod --enable-bcm2835gpio
make -j$(nproc)
make install
popd
rm -rf /tmp/openocd