# ONLY FOR CLUSTER USE
import sys
sys.dont_write_bytecode = True

import subprocess, shlex, os
from pathlib import Path

def main():
    if len(sys.argv) != 2 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print("Usage: >> python3 run_jobs.py <batch_idx>.")
        exit(1)
        
    idx = sys.argv[1]
    batch_folder = f"{Path(__file__).parent.parent}/jobs/batch_{idx}"
    
    # verify inputs
    if input(f"The jobs will be taken from the folder {batch_folder}\nInsert y if it's correct: ") != "y": exit(0)
        
    for job in os.listdir(batch_folder):
        subprocess.run(shlex.split(f"sbatch --wckey=rop --requeue {batch_folder}/{job}"))
        
if __name__ == "__main__":
    main()