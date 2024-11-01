import subprocess
import notify
import shutil
import shlex
import sys
import os
from pathlib import Path

alg = sys.argv[1]
timelimit = sys.argv[2]
instances_folder = sys.argv[3]
cpx_dir = sys.argv[4]
cpx_inc = sys.argv[5]
verbose = sys.argv[6]
warn = sys.argv[7]
intcheck = sys.argv[8]

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
 
def run_batch():
    
    try:

        instances = os.listdir(instances_folder)
        os.chdir(build_dir)

        bot.create_progress_bar(len(instances))
        
        for file in instances:
            
            file_path = os.path.join(instances_folder, file)
            
            cmd = f"./main -a {alg} -l -ln {file}.log -rn {file} -t {timelimit} -i \"{file_path}\""

            try:
                subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
                
            except subprocess.CalledProcessError as e:
                continue
            
            bot.update_progress_bar()
            
        bot.conclude_progress_bar()

    except subprocess.CalledProcessError as e:

        bot.send_message_by_text(e.stderr)
        
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
    
if __name__ == "__main__":
    
    try:
        
        # clear_output_directories()
        # run_batch()
        label_logs()
        
        bot.send_message_by_text("Done.")

    except Exception as e:
        
        bot.send_exception(str(e))