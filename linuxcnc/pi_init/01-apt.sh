#!/usr/bin/env bash

set -e

# various APT repositories
mkdir -p ~/.gnupg
gpg --no-default-keyring --keyring gnupg-ring:/etc/apt/trusted.gpg.d/linuxcnc.gpg --keyserver hkp://keyserver.ubuntu.com --recv-keys e43b5a8e78cc2927
chmod go+r /etc/apt/trusted.gpg.d/linuxcnc.gpg
curl -sS https://repository.qtpyvcp.com/repo/kcjengr.key | gpg --dearmor >/etc/apt/trusted.gpg.d/kcjengr.gpg
curl -sS https://gnipsel.com/flexgui/apt-repo/pgp-key.public >/etc/apt/trusted.gpg.d/flexgui.asc

cat >/etc/apt/sources.list.d/linuxcnc.list <<EOF
#deb [arch=arm64] https://www.linuxcnc.org/ trixie base 2.9-uspace
deb [arch=arm64] https://repository.qtpyvcp.com/apt develop main
deb [arch=arm64] https://gnipsel.com/flexgui/apt-repo stable main
EOF

apt update
apt -y upgrade

# firmware build deps
apt -y install cmake ninja-build gcc-arm-none-eabi