#!/usr/bin/env bash

set -e

mkdir -p ~/linuxcnc/configs
mkdir -p ~/linuxcnc/nc_files

ln -s /usr/share/linuxcnc/ncfiles ~/linuxcnc/nc_files/examples
ln -s /usr/share/linuxcnc/ncfiles/gcmc_lib ~/linuxcnc/nc_files/gcmc_lib
ln -s /usr/share/linuxcnc/ncfiles/gladevcp_lib ~/linuxcnc/nc_files/gladevcp_lib
ln -s /usr/share/linuxcnc/ncfiles/ngcgui_lib ~/linuxcnc/nc_files/ngcgui_lib
ln -s /usr/share/linuxcnc/ncfiles/remap_lib ~/linuxcnc/nc_files/remap_lib

ln -s $PWD/linuxcnc/config/carvera ~/linuxcnc/configs/carvera

cat >>~/linuxcnc/configs/carvera/tool.tbl <<EOF
T99  P99  D+2.000000 Z+0.000000 ;Touch probe
EOF

sudo halcompile --install ./linuxcnc/hal_comps/electrophorus.c
sudo halcompile --install ./linuxcnc/hal_comps/level_map.comp
sudo halcompile --install ./linuxcnc/hal_comps/probe_helper.comp