#!/usr/bin/env bash
# Render a Graphviz .dot file to PNG.
# Usage: ./dot2png.sh <input.dot> [output.png] [layout]
#   layout: dot (default), neato, sfdp, fdp, circo, twopi
set -euo pipefail

if [[ $# -lt 1 ]]; then
    echo "Usage: $0 <input.dot> [output.png] [layout]" >&2
    exit 1
fi

in="$1"
out="${2:-${in%.dot}.png}"
layout="${3:-dot}"

command -v "$layout" >/dev/null || { echo "graphviz layout '$layout' not found (brew install graphviz)" >&2; exit 1; }

"$layout" -Tpng -Gdpi=150 -o "$out" "$in"
echo "wrote $out"
