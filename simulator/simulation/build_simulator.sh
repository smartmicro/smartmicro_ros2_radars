#!/usr/bin/env bash
work_dir=$(pwd)
set -e

cd /code/simulator/simulation

function build_simulation {
    mkdir -p build_dir
    cmake -G "Unix Makefiles" -S. -Bbuild_dir \
    -DCMAKE_INSTALL_PREFIX=out \
    -DSMART_ACCESS_LIB_FOLDER=../libs

    make -C build_dir
    make -C build_dir install
}

build_simulation
