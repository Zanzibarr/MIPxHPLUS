# ONLY FOR CLUSTER USE
import sys

sys.dont_write_bytecode = True

import subprocess, shlex, os


def run_batch(idx: int) -> None:
    batch_folder = os.path.abspath(f"jobs/batch_{idx}")

    # verify inputs
    if (
        input(
            f"The jobs will be taken from the folder {batch_folder}\nInsert y if it's correct: "
        )
        != "y"
    ):
        exit(0)

    for job in os.listdir(batch_folder):
        subprocess.run(
            shlex.split(f"sbatch --wckey=rop --requeue {batch_folder}/{job}")
        )


def main():
    if len(sys.argv) != 2 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print(f"Usage: >> python3 {os.path.basename(__file__)} <batch_idx>.")
        exit(0)

    idx = sys.argv[1]

    if idx == "all":
        for idx in [0, 1, 2, 3]:
            run_batch(idx)
    else:
        run_batch(idx)


if __name__ == "__main__":
    main()
