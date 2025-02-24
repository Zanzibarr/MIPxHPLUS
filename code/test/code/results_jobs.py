# ONLY FOR CLUSTER USE
import sys

sys.dont_write_bytecode = True

import subprocess, shlex, os, json
from pathlib import Path


def move_file(fromfilepath: Path, todir: Path):
    subprocess.run(shlex.split(f"mv {fromfilepath} {todir}/"))


def main():
    runsum = {}

    if len(sys.argv) > 2 or (
        len(sys.argv) == 2 and sys.argv[1] in ["-h", "--h", "-help", "--help"]
    ):
        print(f"Usage: >> python3 {os.path.basename(__file__)} <run_name>.")
        exit(0)

    run_name = "testrun"
    if len(sys.argv) == 2:
        run_name = sys.argv[1]

    # verify inputs
    if input(f"Run name will be '{run_name}'\nInsert y if it's correct: ") != "y":
        exit(0)

    runsum["stats"] = {
        "avg_ptime": 0,
        "avg_stime": 0,
        "avg_htime": 0,
        "avg_btime": 0,
        "avg_cbtime": 0,
        "avg_ctime": 0,
        "avg_time": 0,
        "perc_found": 0,
        "perc_opt": 0,
    }
    runsum["n_total"] = 0
    runsum["other"] = ""
    runsum["results"] = {}

    logsdir = os.path.abspath("../../../logs/AAA_output_logs")
    save_logs_dir = Path(f"{Path(logsdir).parent}/saved_logs/{run_name}")
    if os.path.exists(save_logs_dir):
        print(f"Path {save_logs_dir} already exists.")
        exit(1)
    os.mkdir(save_logs_dir)

    print(f"Logs dir: {logsdir}")
    print(f"Saved logs dir: {save_logs_dir}")
    if input("Check those paths.\nInsert y if it's all correct: ") != "y":
        exit(0)

    n_good = 0

    for file in os.listdir(logsdir):
        filepath = os.path.join(logsdir, file)
        with open(filepath, "r") as f:
            content = f.read()

        if "[ ERROR ]" not in content and (
            "Code version:" not in content
            or int(
                content.partition("Verbose parameter:")[2].partition(".\n")[0].strip()
            )
            < 10
        ):
            print(
                "Instances must be run with a logging parameter of 10 (or higher) for this code to be able to parse all results."
            )
            print(filepath)
            exit(1)

        if "Axiom layer is" in content:
            instance_name = content.partition("RUN_NAME: ")[2].partition("\n")[0]
        else:
            instance_name = Path(
                content.partition("Input file: ")[2].partition(".\n")[0]
            ).name

        instance_name = instance_name.replace(".sas", "").replace("_deletefree", "")

        runsum["results"][instance_name] = {
            "status": -1,
            "hcost": -1,
            "fcost": -1,
            "ptime": -1,
            "stime": -1,
            "htime": -1,
            "btime": -1,
            "cbtime": -1,
            "ctime": -1,
            "time": -1,
            "other": "",
        }

        if "Axiom layer is" in content:
            runsum["results"][instance_name]["status"] = -2
        elif "[ ERROR ]" in content:
            runsum["results"][instance_name]["status"] = -1
        elif "The problem is infeasible." in content:
            runsum["stats"]["perc_found"] += 1
            runsum["stats"]["perc_opt"] += 1
            runsum["results"][instance_name]["status"] = 1
        elif "Reached time limit" in content or "No solution found." in content:
            runsum["results"][instance_name]["status"] = 3
        elif "The solution has not been proven optimal." in content:
            runsum["stats"]["perc_found"] += 1
            runsum["results"][instance_name]["status"] = 2
        elif "Solution cost: " in content:
            runsum["stats"]["perc_found"] += 1
            runsum["stats"]["perc_opt"] += 1
            runsum["results"][instance_name]["status"] = 0
        else:
            runsum["results"][instance_name]["status"] = -1

        if runsum["results"][instance_name]["status"] > -2:
            runsum["n_total"] += 1
            if runsum["results"][instance_name]["status"] > -1:
                n_good += 1
                parsing_time = float(
                    content.partition(">>  Parsing time")[2].partition("s")[0].strip()
                )
                opt_time = float(
                    content.partition(">>  Problem simplification time")[2]
                    .partition("s")[0]
                    .strip()
                )
                heur_time = float(
                    content.partition(">>  Heuristic time")[2].partition("s")[0].strip()
                )
                build_time = float(
                    content.partition(">>  Model building time")[2]
                    .partition("s")[0]
                    .strip()
                )
                callback_time = float(
                    content.partition(">>  CPLEX callback time")[2]
                    .partition("s")[0]
                    .strip()
                )
                cplex_time = float(
                    content.partition(">>  CPLEX execution time")[2]
                    .partition("s")[0]
                    .strip()
                )
                total_time = float(
                    content.partition(">>  Total time")[2].partition("s")[0].strip()
                )
                runsum["stats"]["avg_ptime"] += parsing_time
                runsum["stats"]["avg_stime"] += opt_time
                runsum["stats"]["avg_htime"] += heur_time
                runsum["stats"]["avg_btime"] += build_time
                runsum["stats"]["avg_cbtime"] += callback_time
                runsum["stats"]["avg_ctime"] += cplex_time
                runsum["stats"]["avg_time"] += total_time
                runsum["results"][instance_name]["ptime"] = parsing_time
                runsum["results"][instance_name]["stime"] = opt_time
                runsum["results"][instance_name]["htime"] = heur_time
                runsum["results"][instance_name]["btime"] = build_time
                runsum["results"][instance_name]["cbtime"] = callback_time
                runsum["results"][instance_name]["ctime"] = cplex_time
                runsum["results"][instance_name]["time"] = total_time
                if runsum["results"][instance_name]["status"] == 0:
                    fcost = int(
                        content.partition("Solution cost:")[2]
                        .partition("\n")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["fcost"] = fcost
                if (
                    runsum["results"][instance_name]["status"] not in (1, 3)
                    and "Heuristic:"
                    in content.partition("Problem simplification:")[2].partition(
                        "Time limit:"
                    )[0]
                ):
                    hcost = int(
                        content.partition("Updated best solution - Cost:")[2]
                        .partition(".\n")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["hcost"] = hcost

        move_file(filepath, save_logs_dir)

    runsum["stats"]["avg_ptime"] /= n_good
    runsum["stats"]["avg_stime"] /= n_good
    runsum["stats"]["avg_htime"] /= n_good
    runsum["stats"]["avg_btime"] /= n_good
    runsum["stats"]["avg_cbtime"] /= n_good
    runsum["stats"]["avg_ctime"] /= n_good
    runsum["stats"]["avg_time"] /= n_good
    runsum["stats"]["perc_found"] /= runsum["n_total"]
    runsum["stats"]["perc_opt"] /= runsum["n_total"]

    with open(f"{save_logs_dir}/000_{run_name}.json", "w") as f:
        json.dump(runsum, f, indent=4)


if __name__ == "__main__":
    main()
