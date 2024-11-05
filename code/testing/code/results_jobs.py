import sys
sys.dont_write_bytecode = True
import utils
import subprocess
#import notify
import shlex
import os

os.makedirs(utils.logs_dir, exist_ok=True)
os.makedirs(utils.output_logs_dir, exist_ok=True)
os.makedirs(utils.opt_logs_dir, exist_ok=True)
os.makedirs(utils.good_logs_dir, exist_ok=True)
os.makedirs(utils.timelimit_logs_dir, exist_ok=True)
os.makedirs(utils.b_timelimit_logs_dir, exist_ok=True)
os.makedirs(utils.infease_logs_dir, exist_ok=True)
os.makedirs(utils.errors_logs_dir, exist_ok=True)
os.makedirs(utils.other_logs_dir, exist_ok=True)

#bot = notify.bot()
        
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

#TODO: Get time statistics
def time_stats():
    
    return f"""
TIME STATISTICS TODO
"""

if __name__ == "__main__":
    
#    try:
        
    label_logs()

    data = ""
    data += check_results()
    data += read_logs()
    data += time_stats()

    print(data)
#        bot.send_message_by_text(data)

#    except Exception as e:
        
#        bot.send_exception(str(e))
