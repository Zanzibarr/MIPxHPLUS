#!/bin/bash

cd code || exit
if [[ ! -d build ]]; then
	echo "Build directory doesn't exists, creating one"
	mkdir build
fi
cd build || exit
VERBOSE_OPT=${1:-10}
cmake -D VERBOSE="${VERBOSE_OPT}" ..
make
