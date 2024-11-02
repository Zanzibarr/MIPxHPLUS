import os, sys
import numpy as np
from pathlib import Path

current_dir = Path(__file__).parent
home_dir = current_dir.parent.parent
build_dir = f"{home_dir}/code/build"

time_limit = sys.argv[1]
instances_folder = sys.argv[2]

batch_size = 1000

instances_list = os.listdir(instances_folder)
n_batches = int(np.ceil(len(instances_list)/batch_size))

os.mkdir(f"{current_dir}/imai_jobs")
os.mkdir(f"{current_dir}/rankooh_jobs")

for i in range(n_batches):

    os.mkdir(f"{current_dir}/imai_jobs/batch_{i}")
    os.mkdir(f"{current_dir}/rankooh_jobs/batch_{i}")

    batch_start = i * batch_size

    for j in range(min(batch_size, len(instances_list) - i * batch_size)):

        idx = batch_start + j
        inst = instances_list[idx]
        inst_path = f"{instances_folder}/{inst}"

        job_imai = f"{current_dir}/imai_jobs/batch_{i}/{inst.replace('.sas', '_imai_job.sh')}"
        job_content_imai = f"""#!/bin/bash
#SBATCH --job-name={inst.replace('.sas', '_imai_matzan')}
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
#SBATCH --time=00:20:00
# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

{build_dir}/main -a imai -l -ln {inst.replace('.sas', '_imai')}.log -rn {inst.replace('.sas', '_imai')} -t {time_limit} -i \"{inst_path}\"

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave"""

        with open(job_imai, "w") as f:
            f.write(job_content_imai)
        
        job_rankooh = f"{current_dir}/rankooh_jobs/batch_{i}/{inst.replace('.sas', '_rankooh_job.sh')}"
        job_content_rankooh = f"""#!/bin/bash
#SBATCH --job-name={inst.replace('.sas', '_rankooh_matzan')}
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
#SBATCH --time=00:20:00
# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

{build_dir}/main -a rankooh -l -ln {inst.replace('.sas', '_rankooh')}.log -rn {inst.replace('.sas', '_rankooh')} -t {time_limit} -i \"{inst_path}\"

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave"""

        with open(job_rankooh, "w") as f:
            f.write(job_content_rankooh)