#!/usr/bin/env bash
work_dir=$(pwd)
set -e

function build_simulation {
    mkdir -p build_dir
    cmake -G "Unix Makefiles" -S. -Bbuild_dir \
        -DCMAKE_INSTALL_PREFIX=out \
        -DSMART_ACCESS_LIB_FOLDER=../libs
    
    make -C build_dir
    make -C build_dir install
}

export SMART_ACCESS_CFG_FILE_PATH=/code/umrr11_simulator/config/com_lib_config.json
export LD_LIBRARY_PATH=/code/umrr11_simulator/libs

cd /code/umrr11_simulator/simulation
function run_simulation
{
    ./out/bin/simulator
}

build_simulation
run_simulation
