#!/usr/bin/env bash

TCHAIN_PATH="$(realpath ./tchain.cmake)"

mkdir build_cross
cd build_cross

cmake ../ -DCMAKE_TOOLCHAIN_FILE=$TCHAIN_PATH -DBUILD_EXAMPLES=True -DBUILD_GLSL_EXTENSIONS=False -DBUILD_UNIT_TESTS=False -DBUILD_WITH_TM2=True -DFORCE_RSUSB_BACKEND=True -DCMAKE_BUILD_TYPE=release -DBUILD_EASYLOGGINGPP=True -DBUILD_SHARED_LIBS=True 

make -j8
