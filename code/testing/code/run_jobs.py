import sys
sys.dont_write_bytecode = True
import utils
import subprocess, time, shutil, os
from pathlib import Path
import shlex

if len(sys.argv) < 2:
    print("Missing algorithm.")
    exit(1)

alg = sys.argv[1]

def clear_logs_dir():

    clear_dir(Path(utils.output_logs_dir))
    clear_dir(Path(utils.opt_logs_dir))
    clear_dir(Path(utils.good_logs_dir))
    clear_dir(Path(utils.timelimit_logs_dir))
    clear_dir(Path(utils.b_timelimit_logs_dir))
    clear_dir(Path(utils.infease_logs_dir))
    clear_dir(Path(utils.errors_logs_dir))
    clear_dir(Path(utils.other_logs_dir))
    clear_dir(Path(f"{utils.jobs_folder}/{alg}_jobs/job_outputs"))
    
def clear_dir(directory):
    
    for item in directory.iterdir():
        if item.is_file() or item.is_symlink():
            item.unlink()
        elif item.is_dir():
            shutil.rmtree(item)

def number_pending_jobs() -> int:

    output = subprocess.run(shlex.split("squeue -u $USER"), shell=True, capture_output=True, text=True)
    return int(len(output.stdout.splitlines())) - 1
 
def run():

    alg_jobs_dir = f"{utils.jobs_folder}/{alg}_jobs"
    
    for batch in os.listdir(alg_jobs_dir):

        while(number_pending_jobs() > 500):
            time.sleep(60)
        
        for job in os.listdir(f"{alg_jobs_dir}/{batch}"):
            subprocess.run(f"sbatch --wckey=rop --requeue {alg_jobs_dir}/{batch}/{job}")

if __name__ == "__main__":
        
    # clear_logs_dir()
    # run()
    print(number_pending_jobs())