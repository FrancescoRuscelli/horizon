#!/bin/bash

mkdir build
cd build
cmake ../horizon/cpp -DCMAKE_BUILD_TYPE=Release
make -j4
