#!/bin/bash

cd code
if [ ! -d build ];
then
  echo "Build directory doesn't exists, creating one"
  mkdir build
fi
cd build
VERBOSE_OPT=${1:-10}
cmake -D VERBOSE=$VERBOSE_OPT ..
make