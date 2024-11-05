import sys
sys.dont_write_bytecode = True
import utils
import subprocess
import shlex
import os

os.makedirs(utils.opt_logs_dir, exist_ok=True)
os.makedirs(utils.good_logs_dir, exist_ok=True)
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
        
        if "Testing small instances only." in content or "Axiom layer is" in content:
            move_file(filepath, utils.other_logs_dir)
        elif "[ ERROR ]" in content:
            move_file(filepath, utils.errors_logs_dir)
        elif "The problem is infeasible." in content:
            move_file(filepath, utils.infease_logs_dir)
        elif "Time limit reached while building the model." in content:
            move_file(filepath, utils.b_timelimit_logs_dir)
        elif "No solution found due to time limit." in content:
            move_file(filepath, utils.timelimit_logs_dir)
        elif "The solution is not optimal due to time limit." in content:
            move_file(filepath, utils.good_logs_dir)
        elif "Solution cost: " in content:
            move_file(filepath, utils.opt_logs_dir)
        else:
            move_file(filepath, utils.errors_logs_dir)

#TODO: Compare results with other software to check if the optimal solutions are indeed optimal
def check_results():
    
    return f"""
!! RESULTS HAVEN'T BEEN COMPARED WITH OTHER SOFTWARE YET !!
"""

def read_logs():
    
    heuristic = 0
    optimal = 0
    time_limit = 0
    build_tl = 0
    infeasible = 0
    errors = 0
    other = 0
    total = 0
    
    heuristic += len(os.listdir(utils.good_logs_dir))
    optimal += len(os.listdir(utils.opt_logs_dir))
    infeasible += len(os.listdir(utils.infease_logs_dir))

    time_limit += len(os.listdir(utils.timelimit_logs_dir))
    build_tl += len(os.listdir(utils.b_timelimit_logs_dir))

    errors += len(os.listdir(utils.errors_logs_dir))

    other += len(os.listdir(utils.other_logs_dir))
    
    good = heuristic + optimal + infeasible
    bad = time_limit + build_tl
    total = good + bad + errors
    
    message = f"""
TOTAL INSTANCES: {total}
FOUND A SOLUTION: {good}/{total} ({round(good*100/total, 2)}%)
 -> FOUND THE OPTIMAL: {optimal}/{good} ({round(optimal*100/good, 2)}%)
 -> FOUND FEASIBLE: {heuristic}/{good} ({round(heuristic*100/good, 2)}%)
 -> PROVEN INFEASIBLE: {infeasible}/{good} ({round(infeasible*100/good, 2)}%)
NO SOLUTION FOUND: {bad}/{total} ({round(bad*100/total, 2)}%)
 -> MODEL TOO SLOW: {time_limit}/{bad} ({round(time_limit*100/bad, 2)}%)
 -> BUILD TOO SLOW: {build_tl}/{bad} ({round(build_tl*100/bad, 2)}%)
ERRORS: {errors}/{total} ({round(errors*100/total, 2)}%)
OTHER LOGS: {other}
"""

    return message

def time_stats():

    found_parsing_times = []
    found_optimization_times = []
    found_wst_times = []
    found_build_times = []
    found_execution_times = []

    total_times = []

    for folder in [utils.opt_logs_dir, utils.good_logs_dir]:
        for file in os.listdir(folder):
            with open(f"{folder}/{file}", "r") as f:
                content = f.read()
            
            parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
            opt_time = float(content.partition(">>  Optimization time")[2].partition("s")[0].strip())
            wst_time = float(content.partition(">>  Warm-start time")[2].partition("s")[0].strip())
            build_time = float(content.partition(">>  Build time")[2].partition("s")[0].strip())
            exec_time = float(content.partition(">>  Exec time")[2].partition("s")[0].strip())

            found_parsing_times.append(parsing_time)
            found_optimization_times.append(opt_time)
            found_wst_times.append(wst_time)
            found_build_times.append(build_time)
            found_execution_times.append(exec_time)
            
            total_times.append(parsing_time + opt_time + wst_time + build_time + exec_time)

    timel_parsing_times = []
    timel_optimization_times = []
    timel_wst_times = []
    timel_build_times = []

    for file in os.listdir(utils.timelimit_logs_dir):
        with open(f"{utils.timelimit_logs_dir}/{file}", "r") as f:
            content = f.read()
        
        parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
        opt_time = float(content.partition(">>  Optimization time")[2].partition("s")[0].strip())
        wst_time = float(content.partition(">>  Warm-start time")[2].partition("s")[0].strip())
        build_time = float(content.partition(">>  Build time")[2].partition("s")[0].strip())

        timel_parsing_times.append(parsing_time)
        timel_optimization_times.append(opt_time)
        timel_wst_times.append(wst_time)
        timel_build_times.append(build_time)

    count_parsing = 0
    count_optimization = 0
    count_warm_start = 0
    count_build = 0

    total = len(os.listdir(utils.b_timelimit_logs_dir))

    for file in os.listdir(utils.b_timelimit_logs_dir):
        with open(f"{utils.b_timelimit_logs_dir}/{file}", "r") as f:
            content = f.read()
        
        parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
        opt_time = float(content.partition(">>  Optimization time")[2].partition("s")[0].strip())
        wst_time = float(content.partition(">>  Warm-start time")[2].partition("s")[0].strip())
        build_time = float(content.partition(">>  Build time")[2].partition("s")[0].strip())

        if build_time == 0:
            #if wst_time == 0:
            if opt_time == 0:
                if parsing_time == 0:
                    count_parsing += 1
                else:
                    count_optimization += 1
            else:
                #count_warm_start += 1
                count_build += 1
            #else:
            #    count_build += 1
        else:
            print(f"WTF..., {file}")        
    
    return f"""
SOLUTION FOUND:
 -> Average parsing time: {round(sum(found_parsing_times) / len(found_parsing_times), 4)}
 -> Average optimization time: {round(sum(found_optimization_times) / len(found_optimization_times), 4)}
 -> Average warm-start time: {round(sum(found_wst_times) / len(found_wst_times), 4)}
 -> Average build time: {round(sum(found_build_times) / len(found_build_times), 4)}
 -> Average cplex time: {round(sum(found_execution_times) / len(found_execution_times), 4)}
 -> Average total time: {round(sum(total_times) / len(total_times), 4)}
MODEL TOO SLOW:
 -> Average parsing time: {round(sum(timel_parsing_times) / len(timel_parsing_times), 4)}
 -> Average optimization time: {round(sum(timel_optimization_times) / len(timel_optimization_times), 4)}
 -> Average warm-start time: {round(sum(timel_wst_times) / len(timel_wst_times), 4)}
 -> Average build time: {round(sum(timel_build_times) / len(timel_build_times), 4)}
BUILD TOO SLOW:
 -> Stopped at parsing: {count_parsing} / {total} ({round(count_parsing * 100 / total, 2)})
 -> Stopped at optimization: {count_optimization} / {total} ({round(count_optimization * 100 / total, 2)})
 -> Stopped at warm-start: {count_warm_start} / {total} ({round(count_warm_start * 100 / total, 2)})
 -> Stopped at build: {count_build} / {total} ({round(count_build * 100 / total, 2)})
"""

if __name__ == "__main__":
    
    label_logs()

    data = ""
    data += check_results()
    data += read_logs()
    data += time_stats()

    print(data)