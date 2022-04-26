#!/usr/bin/env bash
work_dir=$(pwd)
set -e

args=("$@")
cd /code/simulator/simulation

function build_simulation {
    mkdir -p build_dir
    cmake -G "Unix Makefiles" -S. -Bbuild_dir \
        -DCMAKE_INSTALL_PREFIX=out \
        -DSMART_ACCESS_LIB_FOLDER=../libs
    
    make -C build_dir
    make -C build_dir install
}

function run_simulation
{
    ./out/bin/simulator ${args[0]} ${args[1]} ${args[2]}
}

build_simulation
run_simulation
