#!/bin/bash

mkdir build
cd build
cmake ../horizon/cpp -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=..
make install -j4