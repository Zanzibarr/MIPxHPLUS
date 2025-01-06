# ONLY FOR CLUSTER USE

import sys
sys.dont_write_bytecode = True

import utils
import os
import random
from pathlib import Path

if len(sys.argv) < 3:
    print("Specify both time limit and instances folder.")
    exit(1)

time_limit = sys.argv[1]
instances_folder = sys.argv[2]

batch_size = 1000

instances_list = os.listdir(instances_folder)
random.shuffle(instances_list)
n_batches = (len(instances_list)//batch_size) + 1

algs = ["imai", "rankooh", "greedy", "dynamic-s", "dynamic-l"]

for alg in algs:

    alg_jobs_folder = f"{utils.jobs_folder}/{alg}_jobs"

    os.makedirs(alg_jobs_folder, exist_ok=True)
    os.makedirs(f"{alg_jobs_folder}/job_outputs", exist_ok=True)
    
    for i in range(n_batches):

        batch_path = f"{alg_jobs_folder}/batch_{i}"

        os.makedirs(batch_path, exist_ok=True)
        utils.clear_dir(Path(batch_path))

        batch_start = i * batch_size

        for j in range(min(batch_size, len(instances_list) - i * batch_size)):

            idx = batch_start + j
            inst = instances_list[idx]
            inst_path = f"{instances_folder}/{inst}"

            job = f"{batch_path}/{inst.replace('.sas', f'_{alg}_job.sh')}"
            job_content = f"""#!/bin/bash
#SBATCH --job-name={inst.replace('.sas', f'_{alg}_matzan')}
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
#SBATCH --time=00:20:00
#SBATCH --output={alg_jobs_folder}/job_outputs/{inst.replace('.sas', f'_{alg}')}.out
#SBATCH --error={alg_jobs_folder}/job_outputs/{inst.replace('.sas', f'_{alg}')}.out

# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

{utils.build_dir}/main -a {alg} -notb -l -ln {inst.replace('.sas', f'_{alg}')}.log -rn {inst.replace('.sas', f'_{alg}')} -t {time_limit} -i \"{inst_path}\"

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave"""

            with open(job, "w") as f:
                f.write(job_content)
