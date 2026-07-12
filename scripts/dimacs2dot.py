#!/usr/bin/env python3
"""Convert a symmetry-graph DIMACS dump (symm_write_graphfile output) to Graphviz dot.

Conventions of the dump: vertices are 1-indexed, facts first then actions.
Colors: 0 = normal fact (implicit, no 'n' line), 1 = goal fact, >= 2 = action
(one color per distinct cost). Node names follow the solver logs:
P<i> (fact), G<i> (goal fact), A<i> (action).

Action fills are derived from the DIMACS color: one evenly-spaced hue per
distinct color class, so actions have the same fill iff they have the same
cost, no matter how many classes there are.

Usage: dimacs2dot.py <dimacs-file> <out.dot>
"""

import sys

src, out = sys.argv[1], sys.argv[2]
colors, edges, n_vertices = {}, [], 0
for line in open(src):
    tok = line.split()
    if not tok:
        continue
    if tok[0] == "p":
        n_vertices = int(tok[2])
    elif tok[0] == "n":
        colors[int(tok[1])] = int(tok[2])
    elif tok[0] == "e":
        edges.append((int(tok[1]), int(tok[2])))

# facts = vertices with color < 2 (color 0 is implicit); actions have color >= 2
n_facts = sum(1 for v in range(1, n_vertices + 1) if colors.get(v, 0) < 2)

# one hue per distinct action color class, evenly spaced around the color wheel
# (offset so a single class comes out blue); HSV strings are native to Graphviz
action_classes = sorted({c for c in colors.values() if c >= 2})
class_fill = {
    c: f"{(0.55 + i / len(action_classes)) % 1.0:.3f} 0.400 0.950"
    for i, c in enumerate(action_classes)
}


def node_id(v):
    if v <= n_facts:
        return f"G{v - 1}" if colors.get(v, 0) == 1 else f"P{v - 1}"
    return f"A{v - 1 - n_facts}"


with open(out, "w") as f:
    f.write(
        'digraph symm {\n  rankdir=LR;\n  node [style=filled, fontname="monospace"];\n'
    )
    for v in range(1, n_vertices + 1):
        c = colors.get(v, 0)
        if c == 0:  # normal fact
            style = 'shape=circle, fillcolor="#e8e8e8"'
        elif c == 1:  # goal fact
            style = 'shape=doublecircle, fillcolor="#ffd92f"'
        else:  # action: fill by cost class, class id in the tooltip (visible in SVG)
            style = f'shape=box, fillcolor="{class_fill[c]}", tooltip="cost class {c}"'
        f.write(f"  {node_id(v)} [{style}];\n")
    for a, b in edges:
        f.write(f"  {node_id(a)} -> {node_id(b)};\n")
    f.write("}\n")
print(
    f"{n_vertices} vertices ({n_facts} facts), {len(edges)} edges, "
    f"{len(action_classes)} action cost class(es) -> {out}"
)
