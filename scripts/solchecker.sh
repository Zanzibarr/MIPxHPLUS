#!/usr/bin/env bash

# Usage: ./solchecker.sh <solution_checker> <instances_path> <solutions_path>
# solution_checker is called in a loop as follows: ./<solution_checker> /path/to/<instance>.sas /path/to/<instance> (first is an instance, second is a solution)
# the instances and solutions are pulled from the specified instances_path and solutions_path
# instances_path expected structure: <instance_name>.sas as Fast Downward instance files
# solutions_path expected structure: <instance_name>.* (any/no extension) as Fast Downward solution files

set -euo pipefail

exec_path="$1"
inst_dir="$2"
sol_dir="$3"

# collect solution files that have a matching instance
sols=()
for sol in "$sol_dir"/*; do
    [ -f "$sol" ] || continue
    name=$(basename "$sol")
    name="${name%.*}"
    [ -f "$inst_dir/$name.sas" ] && sols+=("$sol")
done

total=${#sols[@]}
i=0
for sol in "${sols[@]}"; do
    i=$((i+1))
    name=$(basename "$sol")
    name="${name%.*}"
    inst="$inst_dir/$name.sas"

    echo "[$i/$total] $name"

    output=$("$exec_path" "$inst" "$sol" 1 2>&1) || true

    if ! grep -q "Solution valid \[delete-free true\]: true" <<< "$output"; then
        echo "--- ERROR: $name ---"
        echo "$output"
        echo
    fi
done
