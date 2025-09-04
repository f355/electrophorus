#!/usr/bin/env bash

export USERNAME=$(whoami)

sudo ./linuxcnc/pi_init/01_init.sh
sudo ./linuxcnc/pi_init/02a_headless.sh

mkdir -p ~/linuxcnc/configs
mkdir -p ~/linuxcnc/nc_files

ln -s /usr/share/linuxcnc/ncfiles ~/linuxcnc/nc_files/examples
ln -s /usr/share/linuxcnc/ncfiles/gcmc_lib ~/linuxcnc/nc_files/gcmc_lib
ln -s /usr/share/linuxcnc/ncfiles/gladevcp_lib ~/linuxcnc/nc_files/gladevcp_lib
ln -s /usr/share/linuxcnc/ncfiles/ngcgui_lib ~/linuxcnc/nc_files/ngcgui_lib
ln -s /usr/share/linuxcnc/ncfiles/remap_lib ~/linuxcnc/nc_files/remap_lib

ln -s $PWD/linuxcnc/carvera ~/linuxcnc/configs/carvera

sudo halcompile --install ./linuxcnc/driver/electrophorus.c
sudo halcompile --install ./linuxcnc/driver/level_map.comp
sudo halcompile --install ./linuxcnc/driver/probe_helper.comp
