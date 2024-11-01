from pathlib import Path
import subprocess
import notify       # remember to add notify's path as environment variable before running the tester
import shutil
import shlex
import sys
import os

alg = sys.argv[1]
timelimit = sys.argv[2]
instances_folder = sys.argv[3]

current_dir = Path(__file__).parent
home_dir = current_dir.parent.parent
build_dir = f"{home_dir}/code/build"
logs_dir = f"{home_dir}/logs"
output_logs_dir = f"{logs_dir}/AAA_output_logs"

opt_logs_dir = f"{logs_dir}/AAB_opt_logs"
good_logs_dir = f"{logs_dir}/AAB_good_logs"
timelimit_logs_dir = f"{logs_dir}/AAC_timelimit_logs"
b_timelimit_logs_dir = f"{logs_dir}/AAD_b_timelimit_logs"
infease_logs_dir = f"{logs_dir}/AAE_infeas_logs"
errors_logs_dir = f"{logs_dir}/AAF_errors_logs"
other_logs_dir = f"{logs_dir}/AAF_other_logs"

os.makedirs(build_dir, exist_ok=True)
os.makedirs(logs_dir, exist_ok=True)
os.makedirs(output_logs_dir, exist_ok=True)
os.makedirs(opt_logs_dir, exist_ok=True)
os.makedirs(good_logs_dir, exist_ok=True)
os.makedirs(timelimit_logs_dir, exist_ok=True)
os.makedirs(b_timelimit_logs_dir, exist_ok=True)
os.makedirs(infease_logs_dir, exist_ok=True)
os.makedirs(errors_logs_dir, exist_ok=True)
os.makedirs(other_logs_dir, exist_ok=True)

bot = notify.bot()

def clear_output_directories():
    
    clear_dir(Path(output_logs_dir))
    clear_dir(Path(opt_logs_dir))
    clear_dir(Path(good_logs_dir))
    clear_dir(Path(timelimit_logs_dir))
    clear_dir(Path(b_timelimit_logs_dir))
    clear_dir(Path(infease_logs_dir))
    clear_dir(Path(errors_logs_dir))
    
def clear_dir(directory):
    
    for item in directory.iterdir():
        if item.is_file() or item.is_symlink():
            item.unlink()
        elif item.is_dir():
            shutil.rmtree(item)
 
def run():
    
    os.chdir(build_dir)
    
    instances = os.listdir(instances_folder)

    bot.create_progress_bar(len(instances), f"Testing {alg} on {len(instances)} (timelimit: {timelimit} s):")
    
    for file in instances:
        
        file_path = os.path.join(instances_folder, file)
        
        cmd = f"./main -a {alg} -l -ln {file}.log -rn {file} -t {timelimit} -i \"{file_path}\""
        
        subprocess.run(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        
        bot.update_progress_bar()
        
    bot.conclude_progress_bar()
        
def move_file(frompath, topath):
    
    subprocess.run(shlex.split(f"mv {frompath} {topath}/"))
        
def label_logs():
    
    for file in os.listdir(output_logs_dir):
        
        filepath = os.path.join(output_logs_dir, file)
        
        with open(filepath, "r") as f:
            content = f.read()
        
        if "Testing small instances only." in content or "Axiom layer is" in content:
            move_file(filepath, other_logs_dir)
        elif "[ ERROR ]" in content:
            move_file(filepath, errors_logs_dir)
        elif "The problem is infeasible." in content:
            move_file(filepath, infease_logs_dir)
        elif "Time limit reached while building the model." in content:
            move_file(filepath, b_timelimit_logs_dir)
        elif "No solution found due to time limit." in content:
            move_file(filepath, timelimit_logs_dir)
        elif "The solution is not optimal due to time limit." in content:
            move_file(filepath, good_logs_dir)
        elif "Solution cost: " in content:
            move_file(filepath, opt_logs_dir)
        else:
            move_file(filepath, errors_logs_dir)

#TODO: Compare results with other results to check if the optimal solutions are indeed optimal
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
    
    heuristic += len(os.listdir(good_logs_dir))
    optimal += len(os.listdir(opt_logs_dir))
    infeasible += len(os.listdir(infease_logs_dir))

    time_limit += len(os.listdir(timelimit_logs_dir))
    build_tl += len(os.listdir(b_timelimit_logs_dir))

    errors += len(os.listdir(errors_logs_dir))

    other += len(os.listdir(other_logs_dir))
    
    good = heuristic + optimal + infeasible
    bad = time_limit + build_tl
    total = good + bad + errors
    
    message = f"""
TOTAL INSTANCES: {total}
FOUND A SOLUTION: {good}/{total} ({round(good*100/total, 2)}%)
 -> FOUND THE OPTIMAL: {optimal}/{good} ({round(optimal*100/good, 2)}%)
 -> FOUND FEASIBLE: {heuristic}/{good} ({round(heuristic*100/good, 2)}%)
 -> PROVEN INFEASIBLE: {infeasible}/{good} ({round(infeasible*100/total, 2)}%)
NO SOLUTION FOUND: {bad}/{total} ({round(bad*100/total, 2)}%)
 -> MODEL TOO SLOW: {time_limit}/{bad} ({round(time_limit*100/bad, 2)}%)
 -> BUILD TOO SLOW: {build_tl}/{bad} ({round(build_tl*100/bad, 2)}%)
ERRORS: {errors}/{total} ({round(errors*100/total, 2)}%)
OTHER LOGS: {other}
"""

    return message

#TODO: Get time statistics
def time_stats():
    
    return f"""
TIME STATISTICS TODO
"""

if __name__ == "__main__":
    
    try:
        
        clear_output_directories()
        run()
        label_logs()
        
        data = ""
        data += check_results()
        data += read_logs()
        data += time_stats()
        
        bot.send_message_by_text(data)

    except Exception as e:
        
        bot.send_exception(str(e))