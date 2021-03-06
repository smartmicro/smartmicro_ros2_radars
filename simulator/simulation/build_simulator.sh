#!/usr/bin/env bash
work_dir=$(pwd)
set -e

cd /code/simulator/simulation

mkdir -p build_dir
cmake -G "Unix Makefiles" -S. -Bbuild_dir \
-DCMAKE_INSTALL_PREFIX=out \
-DSMART_ACCESS_LIB_FOLDER=../../umrr_ros2_driver/smartmicro/lib-linux-x86_64-gcc_9
make -C build_dir
make -C build_dir install
