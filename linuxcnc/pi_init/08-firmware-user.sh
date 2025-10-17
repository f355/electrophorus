#!/usr/bin/env bash

set -e

git submodule init
git submodule update --depth 1

mkdir -p cmake-build-release
cmake -Bcmake-build-release -GNinja -DCMAKE_BUILD_TYPE=Release -DMBED_TARGET=LPC1768
ninja -C cmake-build-release