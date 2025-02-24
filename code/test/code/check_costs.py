import sys

sys.dont_write_bytecode = True

import os, json


def main():
    errors = 0
    errors_names = ""
    total = 0

    if len(sys.argv) != 2 or sys.argv[1] in ["-h", "--h", "-help", "--help"]:
        print(
            f"Usage: >> python3 {os.path.basename(__file__)} <path_to_run_results.json>."
        )
        exit(0)

    run_results_file = os.path.abspath(sys.argv[1])

    if (
        input(f"Run to check will be '{run_results_file}'\nInsert y if it's correct: ")
        != "y"
    ):
        exit(0)

    with open(run_results_file, "r") as f:
        run_results = json.loads(f.read())

    baseline_path = os.path.abspath("../../../local/fast_downward_baseline")
    if (
        input(f"Fast downward baseline: {baseline_path}\nInsert y if it's correct: ")
        != "y"
    ):
        exit(0)

    for file in os.listdir(baseline_path):
        file_path = os.path.join(baseline_path, file)
        with open(file_path, "r") as f:
            content = f.read()

        if file not in run_results["results"]:
            continue

        inst_res = run_results["results"][file]

        bound = content.splitlines()[0].split(" ")
        bound = (float(bound[0]), float(bound[1]))

        if inst_res["status"] < 0:
            continue

        elif (
            (
                inst_res["status"] == 0
                and (inst_res["fcost"] < bound[0] or inst_res["fcost"] > bound[1])
            )
            or (inst_res["status"] == 1 and content.splitlines()[0] != "0.0 1e+20")
            or (inst_res["status"] == 2 and inst_res["hcost"] < bound[0])
        ):
            errors += 1
            errors_names += f"{file}\n"

        total += 1

    print(f"Total checked: {total}")
    print(f"Total errors: {errors}")
    if errors != 0:
        print(errors_names)


if __name__ == "__main__":
    main()
