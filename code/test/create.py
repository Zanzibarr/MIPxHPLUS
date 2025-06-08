# ONLY FOR CLUSTER USE
import sys

sys.dont_write_bytecode = True

import os, random, shutil
from pathlib import Path


def clear_dir(dir: Path):
    for item in dir.iterdir():
        if item.is_file() or item.is_symlink():
            item.unlink()
        elif item.is_dir():
            shutil.rmtree(item)


def main():
    # verify input correctness
    if len(sys.argv) <= 1 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print(
            f"Usage: >> python3 {os.path.basename(__file__)} <instances_folder> <all parameters to pass to the execution>."
        )
        exit(0)

    # read inputs
    instances_folder = os.path.abspath(sys.argv[1])
    execution_parameters = " ".join(sys.argv[2:])
    example_parameters = (
        execution_parameters + " --t=900 --mem=16000 --log=<instance_name>.log"
    )

    # verify inputs
    if (
        input(
            f"The instances will be taken from the folder {instances_folder}\nInsert y if it's correct: "
        )
        != "y"
    ):
        exit(0)
    if (
        input(
            f"The execution will have the following structure:\n>> ./hplus --run <instance> {example_parameters}\nInsert y if it's correct: "
        )
        != "y"
    ):
        exit(0)

    # organize batches
    batch_size = 1000
    instances_list = os.listdir(instances_folder)
    random.shuffle(instances_list)
    n_batches = (len(instances_list) // batch_size) + 1

    code_dir = os.path.abspath("..")

    # jobs
    jobs_folder = os.path.join(code_dir, "test/jobs")
    jobs_outputs = os.path.join(code_dir, "test/jobs_output")

    exec_dir = os.path.join(code_dir, "build")

    print(f"Jobs folder: {jobs_folder}")
    print(f"Jobs output: {jobs_outputs}")
    print(f"Executable: {exec_dir}/hplus")
    if input("Check those paths.\nInsert y if it's all correct: ") != "y":
        exit(0)

    os.makedirs(jobs_folder, exist_ok=True)
    os.makedirs(jobs_outputs, exist_ok=True)
    clear_dir(Path(jobs_outputs))

    for i in range(n_batches):
        batch_path = f"{jobs_folder}/batch_{i}"
        os.makedirs(batch_path, exist_ok=True)
        clear_dir(Path(batch_path))

        batch_start = i * batch_size

        for j in range(min(batch_size, len(instances_list) - batch_start)):
            idx = batch_start + j
            inst = instances_list[idx]
            inst_name = inst.replace(".sas", "")
            inst_path = f"{instances_folder}/{inst}"
            job = f"{batch_path}/{inst.replace('.sas', '_job.sh')}"
            job_content = f"""#!/bin/bash
#SBATCH --job-name={inst.replace('.sas', '_matzan')}
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
#SBATCH --time=00:20:00
#SBATCH --output={jobs_outputs}/{inst_name}.out
#SBATCH --error={jobs_outputs}/{inst_name}.out

# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

{exec_dir}/./hplus --run {inst_path} {execution_parameters} --t=900 --mem=16000 --log={inst_name}.log

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave"""

            with open(job, "w") as f:
                f.write(job_content)


if __name__ == "__main__":
    main()
