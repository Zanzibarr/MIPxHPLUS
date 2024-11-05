import sys
sys.dont_write_bytecode = True
import utils
import subprocess, time, shutil, os
from pathlib import Path
import shlex

if len(sys.argv) < 3:
    print("Missing algorithm and batch index.")
    exit(1)

alg = sys.argv[1]
batch_idx = sys.argv[2]

def run():

    alg_jobs_dir = f"{utils.jobs_folder}/{alg}_jobs"

    input(f"{alg_jobs_dir}/{batch_idx}")

    for job in os.listdir(f"{alg_jobs_dir}/{batch_idx}"):
        subprocess.run(shlex.split(f"sbatch --wckey=rop --requeue {alg_jobs_dir}/{batch_idx}/{job}"))

if __name__ == "__main__":
    run()
