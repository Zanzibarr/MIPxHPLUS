#!/bin/bash

cd ../code/build/
./hplus -i "$1" -v "${@:2}"
cd ../../scripts/
python3 dimacs2dot.py ../code/build/test.txt g.dot
dot -Tsvg g.dot -o g.svg