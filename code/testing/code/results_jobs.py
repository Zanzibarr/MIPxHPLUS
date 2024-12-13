# ONLY FOR CLUSTER USE

import sys
sys.dont_write_bytecode = True

import utils
import subprocess
import shlex
import os
from pathlib import Path
import json

timelimit = 900

runsum = {}

runfile = "run_file.json"
if len(sys.argv) > 1: runfile = sys.argv[1]

runsum["stats"] = {
    "avg_ptime" : 0,
    "avg_stime" : 0,
    "avg_htime" : 0,
    "avg_btime" : 0,
    "avg_ctime" : 0,
    "avg_time" : 0,
    "perc_found" : 0,
    "perc_opt" : 0,
    "perc_nfound" : 0
}
runsum["ntotal"] = 0
runsum["other"] = ""
runsum["results"] = {}

os.makedirs(utils.opt_logs_dir, exist_ok=True)
os.makedirs(utils.feas_logs_dir, exist_ok=True)
os.makedirs(utils.timelimit_logs_dir, exist_ok=True)
os.makedirs(utils.b_timelimit_logs_dir, exist_ok=True)
os.makedirs(utils.infease_logs_dir, exist_ok=True)
os.makedirs(utils.errors_logs_dir, exist_ok=True)
os.makedirs(utils.other_logs_dir, exist_ok=True)

def move_file(frompath, topath):

    subprocess.run(shlex.split(f"mv {frompath} {topath}/"))

def label_logs():

    for file in os.listdir(utils.output_logs_dir):

        filepath = os.path.join(utils.output_logs_dir, file)

        with open(filepath, "r") as f:
            content = f.read()

        instance_name = Path(content.partition("Input file: ")[2].partition(".\n")[0]).name.replace(".sas", "")
        runsum["results"][instance_name] = {
            "status" : -1,
            "wscost" : -1,
            "fcost" : -1,
            "ptime" : -1,
            "stime" : -1,
            "htime" : -1,
            "btime" : -1,
            "ctime" : -1,
            "time" : -1,
            "other" : "none"
        }

        if "Testing small instances only." in content or "Axiom layer is" in content:
            runsum["results"][instance_name]["status"] = -2
            move_file(filepath, utils.other_logs_dir)
        elif "[ ERROR ]" in content:
            move_file(filepath, utils.errors_logs_dir)
        elif "The problem is infeasible." in content:
            runsum["stats"]["perc_found"] += 1
            runsum["stats"]["perc_opt"] += 1
            runsum["results"][instance_name]["status"] = 1
            move_file(filepath, utils.infease_logs_dir)
        elif "Reached time limit" in content and "Reached time limit during CPLEX's execution." not in content:
            runsum["stats"]["perc_nfound"] += 1
            runsum["results"][instance_name]["status"] = 4
            move_file(filepath, utils.b_timelimit_logs_dir)
        elif "No solution found." in content:
            runsum["stats"]["perc_nfound"] += 1
            runsum["results"][instance_name]["status"] = 3
            move_file(filepath, utils.timelimit_logs_dir)
        elif "The solution has not been proven optimal." in content:
            runsum["stats"]["perc_found"] += 1
            runsum["results"][instance_name]["status"] = 2
            move_file(filepath, utils.feas_logs_dir)
        elif "Solution cost: " in content:
            runsum["stats"]["perc_found"] += 1
            runsum["stats"]["perc_opt"] += 1
            runsum["results"][instance_name]["status"] = 0
            move_file(filepath, utils.opt_logs_dir)
        else:
            move_file(filepath, utils.errors_logs_dir)

        if runsum["results"][instance_name]["status"] > -2:
            runsum["ntotal"] += 1

    runsum["stats"]["perc_found"] /= runsum["ntotal"]
    runsum["stats"]["perc_opt"] /= runsum["ntotal"]
    runsum["stats"]["perc_nfound"] /= runsum["ntotal"]

def check_results():

    errors = 0
    total = 0

    for file in os.listdir(utils.opt_logs_dir):
        with open(f"{utils.opt_logs_dir}/{file}", "r") as f:
            content = f.read()

        cost = int(content.partition("Solution cost:")[2].partition("\n")[0].strip())

        instance_path = content.partition("Input file: ")[2].partition(".\n")[0]
        instance_name = Path(instance_path).name.replace(".sas", "")
        if not os.path.exists(f"{utils.home_dir}/local/fast_downward_baseline/{instance_name}"): continue

        total += 1

        with open(f"{utils.home_dir}/local/fast_downward_baseline/{instance_name}", "r") as f:
            content_baseline = f.read().splitlines()

        cost_range = [float(x) for x in content_baseline[0].split(" ")]

        if cost < cost_range[0] or cost > cost_range[1]:
            print(f"ERROR: {instance_name}.")
            errors += 1

    return f"""
RESULTS COMPARED WITH FAST DOWNWARD RANKOOH IMPLEMENTATION:
ERRORS: {errors} / {total} ({round(errors*100/total, 2)}%)
"""

def read_logs():

    feasible = 0
    optimal = 0
    time_limit = 0
    build_tl = 0
    infeasible = 0
    errors = 0
    other = 0
    total = 0

    feasible += len(os.listdir(utils.feas_logs_dir))
    optimal += len(os.listdir(utils.opt_logs_dir))
    infeasible += len(os.listdir(utils.infease_logs_dir))

    time_limit += len(os.listdir(utils.timelimit_logs_dir))
    build_tl += len(os.listdir(utils.b_timelimit_logs_dir))

    errors += len(os.listdir(utils.errors_logs_dir))

    other += len(os.listdir(utils.other_logs_dir))

    good = feasible + optimal + infeasible
    bad = time_limit + build_tl
    total = good + bad + errors

    message = f"""
TOTAL INSTANCES: {total}
FOUND A SOLUTION: {good}/{total} ({round(good*100/total, 2)}%)
 -> FOUND THE OPTIMAL: {optimal}/{good} ({round(optimal*100/good, 2)}%)
 -> FOUND FEASIBLE: {feasible}/{good} ({round(feasible*100/good, 2)}%)
 -> PROVEN INFEASIBLE: {infeasible}/{good} ({round(infeasible*100/good, 2)}%)
NO SOLUTION FOUND: {bad}/{total} ({round(bad*100/total, 2)}%)
 -> MODEL TOO SLOW: {time_limit}/{bad} ({round(time_limit*100/bad, 2)}%)
 -> BUILD TOO SLOW: {build_tl}/{bad} ({round(build_tl*100/bad, 2)}%)
ERRORS: {errors}/{total} ({round(errors*100/total, 2)}%)
OTHER LOGS: {other}
"""

    return message

def time_stats():

    feas_parsing_times = []
    feas_prob_simpl_times = []
    feas_heur_times = []
    feas_build_times = []
    feas_execution_times = []
    feas_total_times = []

    for file in os.listdir(utils.feas_logs_dir):
        with open(f"{utils.feas_logs_dir}/{file}", "r") as f:
            content = f.read()

        parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
        simplification_time = float(content.partition(">>  Problem simplification time")[2].partition("s")[0].strip())
        heur_time = float(content.partition(">>  Heuristic time")[2].partition("s")[0].strip())
        build_time = float(content.partition(">>  Model building time")[2].partition("s")[0].strip())
        exec_time = float(content.partition(">>  CPLEX execution time")[2].partition("s")[0].strip())
        total_time = float(content.partition(">>  Total time")[2].partition("s")[0].strip())

        feas_parsing_times.append(parsing_time)
        feas_prob_simpl_times.append(simplification_time)
        feas_heur_times.append(heur_time)
        feas_build_times.append(build_time)
        feas_execution_times.append(exec_time)
        feas_total_times.append(total_time)

        instance_name = Path(content.partition("Input file: ")[2].partition(".\n")[0]).name.replace(".sas", "")

        runsum["results"][instance_name]["ptime"] = parsing_time
        runsum["results"][instance_name]["stime"] = simplification_time
        runsum["results"][instance_name]["htime"] = heur_time
        runsum["results"][instance_name]["btime"] = build_time
        runsum["results"][instance_name]["ctime"] = exec_time
        runsum["results"][instance_name]["time"] = total_time

        if content.partition("Warm start:")[2].partition(".\n")[0] in ["Y", "enabled"]:
            if "Updated best solution - Cost:" in content:
                runsum["results"][instance_name]["wscost"] = int(content.partition("Updated best solution - Cost:")[2].partition(".\n")[0])

        if "Solution cost:" in content: runsum["results"][instance_name]["fcost"] = int(content.partition("Solution cost:")[2].partition("\n")[0])

    optimal_parsing_times = []
    optimal_prob_simpl_times = []
    optimal_heur_times = []
    optimal_build_times = []
    optimal_execution_times = []
    optimal_total_times = []

    for file in os.listdir(utils.opt_logs_dir):
        with open(f"{utils.opt_logs_dir}/{file}", "r") as f:
            content = f.read()

        parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
        simplification_time = float(content.partition(">>  Problem simplification time")[2].partition("s")[0].strip())
        heur_time = float(content.partition(">>  Heuristic time")[2].partition("s")[0].strip())
        build_time = float(content.partition(">>  Model building time")[2].partition("s")[0].strip())
        exec_time = float(content.partition(">>  CPLEX execution time")[2].partition("s")[0].strip())
        total_time = float(content.partition(">>  Total time")[2].partition("s")[0].strip())

        optimal_parsing_times.append(parsing_time)
        optimal_prob_simpl_times.append(simplification_time)
        optimal_heur_times.append(heur_time)
        optimal_build_times.append(build_time)
        optimal_execution_times.append(exec_time)
        optimal_total_times.append(total_time)

        instance_name = Path(content.partition("Input file: ")[2].partition(".\n")[0]).name.replace(".sas", "")

        runsum["results"][instance_name]["ptime"] = parsing_time
        runsum["results"][instance_name]["stime"] = simplification_time
        runsum["results"][instance_name]["htime"] = heur_time
        runsum["results"][instance_name]["btime"] = build_time
        runsum["results"][instance_name]["ctime"] = exec_time
        runsum["results"][instance_name]["time"] = total_time

        if content.partition("Warm start:")[2].partition(".\n")[0] in ["Y", "enabled"]:
            if "Updated best solution - Cost:" in content:
                runsum["results"][instance_name]["wscost"] = int(content.partition("Updated best solution - Cost:")[2].partition(".\n")[0])

        if "Solution cost:" in content: runsum["results"][instance_name]["fcost"] = int(content.partition("Solution cost:")[2].partition("\n")[0])

    timel_parsing_times = []
    timel_prob_simpl_times = []
    timel_heur_times = []
    timel_build_times = []

    for file in os.listdir(utils.timelimit_logs_dir):
        with open(f"{utils.timelimit_logs_dir}/{file}", "r") as f:
            content = f.read()

        parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
        simplification_time = float(content.partition(">>  Problem simplification time")[2].partition("s")[0].strip())
        heur_time = float(content.partition(">>  Heuristic time")[2].partition("s")[0].strip())
        build_time = float(content.partition(">>  Model building time")[2].partition("s")[0].strip())

        timel_parsing_times.append(parsing_time)
        timel_prob_simpl_times.append(simplification_time)
        timel_heur_times.append(heur_time)
        timel_build_times.append(build_time)

        instance_name = Path(content.partition("Input file: ")[2].partition(".\n")[0]).name.replace(".sas", "")

        runsum["results"][instance_name]["ptime"] = parsing_time
        runsum["results"][instance_name]["stime"] = simplification_time
        runsum["results"][instance_name]["htime"] = heur_time
        runsum["results"][instance_name]["btime"] = build_time
        runsum["results"][instance_name]["ctime"] = timelimit - parsing_time + simplification_time + heur_time + build_time
        runsum["results"][instance_name]["time"] = timelimit

        if content.partition("Warm start:")[2].partition(".\n")[0] in ["Y", "enabled"]:
            if "Updated best solution - Cost:" in content:
                runsum["results"][instance_name]["wscost"] = int(content.partition("Updated best solution - Cost:")[2].partition(".\n")[0])

    count_btl_parsing = 0
    count_btl_prob_simpl = 0
    count_btl_heur = 0
    count_btl_build = 0

    total = len(os.listdir(utils.b_timelimit_logs_dir))

    for file in os.listdir(utils.b_timelimit_logs_dir):
        with open(f"{utils.b_timelimit_logs_dir}/{file}", "r") as f:
            content = f.read()

        parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
        simplification_time = float(content.partition(">>  Problem simplification time")[2].partition("s")[0].strip())
        heur_time = float(content.partition(">>  Heuristic time")[2].partition("s")[0].strip())

        instance_name = Path(content.partition("Input file: ")[2].partition(".\n")[0]).name.replace(".sas", "")

        runsum["results"][instance_name]["ptime"] = parsing_time
        runsum["results"][instance_name]["stime"] = simplification_time
        runsum["results"][instance_name]["htime"] = heur_time
        runsum["results"][instance_name]["btime"] = timelimit - parsing_time + simplification_time + heur_time
        runsum["results"][instance_name]["ctime"] = timelimit - parsing_time + simplification_time + heur_time
        runsum["results"][instance_name]["time"] = timelimit

        if content.partition("Warm start:")[2].partition(".\n")[0] in ["Y", "enabled"]:
            if "Updated best solution - Cost:" in content:
                runsum["results"][instance_name]["wscost"] = int(content.partition("Updated best solution - Cost:")[2].partition(".\n")[0])
        
        if "Reached time limit while parsing the instance file." in content:
            count_btl_parsing += 1
        elif "Reached time limit while simplificating the problem." in content:
            count_btl_prob_simpl += 1
        elif "Reached time limit while calculating an heuristic solution." in content:
            count_btl_heur += 1
        elif "Reached time limit while building the model." in content:
            count_btl_build += 1
            
        else:
            print(f"WTF..., {file}")

    msg = ""

    if len(optimal_parsing_times) > 0: msg += f"""
OPT SOLUTION FOUND:
 -> Average parsing time: {round(sum(optimal_parsing_times) / len(optimal_parsing_times), 4)}s
 -> Average problem simplification time: {round(sum(optimal_prob_simpl_times) / len(optimal_prob_simpl_times), 4)}s
 -> Average heuristic time: {round(sum(optimal_heur_times) / len(optimal_heur_times), 4)}s
 -> Average model building time: {round(sum(optimal_build_times) / len(optimal_build_times), 4)}s
 -> Average CPLEX execution time: {round(sum(optimal_execution_times) / len(optimal_execution_times), 4)}s
 -> Average total time: {round(sum(optimal_total_times) / len(optimal_total_times), 4)}s
"""
    if len(feas_parsing_times) > 0: msg += f"""A SOLUTION FOUND:
 -> Average parsing time: {round(sum(feas_parsing_times) / len(feas_parsing_times), 4)}s
 -> Average problem simplification time: {round(sum(feas_prob_simpl_times) / len(feas_prob_simpl_times), 4)}s
 -> Average heuristic time: {round(sum(feas_heur_times) / len(feas_heur_times), 4)}s
 -> Average model building time: {round(sum(feas_build_times) / len(feas_build_times), 4)}s
 -> Average CPLEX execution time: {round(sum(feas_execution_times) / len(feas_execution_times), 4)}s
 -> Average total time: {round(sum(feas_total_times) / len(feas_total_times), 4)}s
"""
    if len(timel_parsing_times) > 0: msg += f"""MODEL TOO SLOW:
 -> Average parsing time: {round(sum(timel_parsing_times) / len(timel_parsing_times), 4)}s
 -> Average problem simplification time: {round(sum(timel_prob_simpl_times) / len(timel_prob_simpl_times), 4)}s
 -> Average heuristic time: {round(sum(timel_heur_times) / len(timel_heur_times), 4)}s
 -> Average model building time: {round(sum(timel_build_times) / len(timel_build_times), 4)}s
"""
    if total > 0: msg += f"""BUILD TOO SLOW:
 -> Stopped during parsing: {count_btl_parsing} / {total} ({round(count_btl_parsing * 100 / total, 2)}%)
 -> Stopped during problem simplification: {count_btl_prob_simpl} / {total} ({round(count_btl_prob_simpl * 100 / total, 2)}%)
 -> Stopped during heuristic: {count_btl_heur} / {total} ({round(count_btl_heur * 100 / total, 2)}%)
 -> Stopped during model building: {count_btl_build} / {total} ({round(count_btl_build * 100 / total, 2)}%)
"""

    return msg

if __name__ == "__main__":

    label_logs()

    data = ""
    data += check_results()
    data += read_logs()
    data += time_stats()

    print(data)

    with open(runfile, "w") as f:
        json.dump(runsum, f, indent=4)
