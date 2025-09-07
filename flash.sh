#!/usr/bin/env bash

set -e

git submodule init
git submodule update

mkdir -p build
cd build
cmake .. -GNinja -DCMAKE_BUILD_TYPE=Release -DMBED_TARGET=LPC1768
sudo ninja flash-electrophorus
