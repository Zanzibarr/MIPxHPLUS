#!/bin/bash

p_flag=(-p 0)
[ "$2" = "1" ] && p_flag=()

cd ../code/build/
./hplus -i "$1" -v "${p_flag[@]}"
cd ../../scripts/
python3 dimacs2dot.py ../code/build/test.txt g.dot
dot -Tsvg g.dot -o g.svg