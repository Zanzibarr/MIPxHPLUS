#!/bin/bash

python3 dimacs2dot.py ../code/build/test.txt g.dot

dot -Tsvg g.dot -o g.svg