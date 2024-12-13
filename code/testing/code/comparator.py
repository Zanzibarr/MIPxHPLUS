# ONLY FOR CLUSTER USE

# TODO: Redo with the new json summary of the run

import sys
sys.dont_write_bytecode = True

import os
from pathlib import Path
import json

if len(sys.argv) < 3:
    print("Needed at least two paths.")
    exit(1)

if any(not os.path.isdir(x) for x in sys.argv[1:]):
    print("Paths specified must be existing paths.")
    exit(1)

logs_dirs = ["/AAB_opt_logs", "/AAB_good_logs", "/AAC_timelimit_logs", "/AAD_b_timelimit_logs"]

times_costs_dict = {}

n_runs = len(sys.argv[1:])
i_run = 0

for run_dir in sys.argv[1:]:
    for log_dir in logs_dirs:

        directory = run_dir+log_dir

        if not os.path.isdir(directory): print(f"{directory} not found.")

        for file in os.listdir(directory):
            with open(f"{directory}/{file}", "r") as f:
                content = f.read()

            instance_name = Path(content.partition("Input file:")[2].partition(".\n")[0]).name.replace(".sas", "")

            if instance_name not in times_costs_dict:
                times_costs_dict[instance_name]["costs"] = [-1]*n_runs 
                times_costs_dict[instance_name]["times"] = [-1]*n_runs
            
            if "Solution cost:" in content:
                solution_cost = int(content.partition("Solution cost:")[2].partition("\n")[0])
                times_costs_dict[instance_name]["costs"][i_run] = solution_cost
            
            total_time = float(content.partition(">>  Total time")[2].partition("s")[0].strip())
            times_costs_dict[instance_name]["times"][i_run] = total_time

    i_run += 1

with open("comparator_data.json", "w") as file:
    json.dump(times_costs_dict, file, indent=4)
