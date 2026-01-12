#!/bin/bash

cmake -S submodules/jsbsim -B build/jsbsim-release \
  -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_SHARED_LIBS=OFF

cmake --build build/jsbsim-release --config Release

scons platform=linux