# ONLY FOR CLUSTER USE
import sys

sys.dont_write_bytecode = True

import subprocess, shlex, os, json, re
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

    logsdir = os.path.abspath("../../../logs/output_logs")
    cpxlogsdir = os.path.abspath("../../../logs/cpxout/log")
    save_logs_dir = Path(f"{Path(logsdir).parent}/saved_logs/{run_name}")
    save_cpxlogs_dir = Path(f"{Path(logsdir).parent}/saved_logs/{run_name}/cpxout")
    if os.path.exists(save_logs_dir):
        print(f"Path {save_logs_dir} already exists.")
        exit(1)

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
    os.mkdir(save_logs_dir)
    os.mkdir(save_cpxlogs_dir)

    print(f"Logs dir: {logsdir}")
    print(f"CPX logs dir: {cpxlogsdir}")
    print(f"Saved logs dir: {save_logs_dir}")
    print(f"Saved cpx logs dir: {save_cpxlogs_dir}")
    if input("Check those paths.\nInsert y if it's all correct: ") != "y":
        exit(0)

    n_good = 0

    for file in os.listdir(logsdir):
        filepath = os.path.join(logsdir, file)
        with open(filepath, "r") as f:
            content = f.read()

        instance_name = (
            content.partition("RUN_NAME: ")[2]
            .partition("\n")[0]
            .replace(".sas", "")
            .replace("_deletefree", "")
        )

        runsum["results"][instance_name] = {
            "status": -1,
            "hcost": -1,
            "fcost": -1,
            "nnodes": -1,
            "nvar_base": -1,
            "nvar_acyc": -1,
            "nconst_base": -1,
            "nconst_acyc": -1,
            "lb": -1,
            "incumb": [],
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
        elif "OUT OF MEMORY" in content:
            runsum["results"][instance_name]["status"] = -3
        elif "[ ERR  ]" in content:
            runsum["results"][instance_name]["status"] = -1
            runsum["results"][instance_name]["other"] = "\n".join(
                content.splitlines()[-5:]
            )
        elif "Statistics" in content:
            runsum["results"][instance_name]["status"] = int(
                content.partition(">>  Status")[2].partition("<<")[0].strip()
            )
            if runsum["results"][instance_name]["status"] in (0, 1):
                runsum["stats"]["perc_found"] += 1
                runsum["stats"]["perc_opt"] += 1
            elif runsum["results"][instance_name]["status"] == 2:
                runsum["stats"]["perc_found"] += 1
        else:
            runsum["results"][instance_name]["status"] = -1
            runsum["results"][instance_name]["other"] = "\n".join(
                content.splitlines()[-5:]
            )

        if runsum["results"][instance_name]["status"] > -2:
            runsum["n_total"] += 1
            if runsum["results"][instance_name]["status"] > -1:
                n_good += 1
                parsing_time = float(
                    content.partition(">>  Parsing time")[2].partition("s")[0].strip()
                )
                opt_time = float(
                    content.partition(">>  Preprocessing time")[2]
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

                # If I have a solution (status 0 = opt, status 2 = heur)
                if (
                    runsum["results"][instance_name]["status"] not in (1, 3)
                    and "Heuristic:"
                    in content.partition("Preprocessing:")[2].partition("Time limit:")[
                        0
                    ]
                ):
                    runsum["results"][instance_name]["hcost"] = int(
                        content.partition(">>  Heur cost")[2].partition("<<")[0].strip()
                    )

                # Find the final cost proposed by the algorithm
                if runsum["results"][instance_name]["status"] not in (1, 3):
                    runsum["results"][instance_name]["fcost"] = int(
                        content.partition(">>  Final cost")[2]
                        .partition("<<")[0]
                        .strip()
                    )

                if "Running CPLEX." in content:
                    runsum["results"][instance_name]["nnodes"] = int(
                        content.partition(">>  Nodes expanded (cplex)")[2]
                        .partition("<<")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["nvar_base"] = int(
                        content.partition(">>  Base model variables (cplex)")[2]
                        .partition("<<")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["nvar_acyc"] = int(
                        content.partition(">>  Acyclicity variables (cplex)")[2]
                        .partition("<<")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["nconst_base"] = int(
                        content.partition(">>  Base model const. (cplex)")[2]
                        .partition("<<")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["nconst_acyc"] = int(
                        content.partition(">>  Acyclicity const. (cplex)")[2]
                        .partition("<<")[0]
                        .strip()
                    )
                    runsum["results"][instance_name]["lb"] = float(
                        content.partition(">>  Lower bound (cplex)")[2]
                        .partition("<<")[0]
                        .strip()
                    )

                cpxlogfile = f"{cpxlogsdir}/{file}"
                if os.path.isfile(cpxlogfile):
                    with open(cpxlogfile, "r") as f:
                        cpxlogcontent = f.read()
                    if runsum["results"][instance_name]["hcost"] >= 0:
                        runsum["results"][instance_name]["incumb"].append(
                            [float(0), runsum["results"][instance_name]["hcost"]]
                        )

                    # Find all matches in the log content
                    incumbent_pattern = re.compile(
                        r"Found incumbent of value (\d+\.\d+) after (\d+\.\d+) sec\."
                    )
                    counter = 0
                    for match in incumbent_pattern.finditer(cpxlogcontent):
                        # Extract value and time, convert to float
                        value = float(match.group(1))
                        time = float(match.group(2))

                        if (
                            counter == 0
                            and value > runsum["results"][instance_name]["hcost"]
                        ):
                            runsum["results"][instance_name]["incumb"] = []

                        counter += 1

                        # Append pair to the list
                        runsum["results"][instance_name]["incumb"].append([time, value])

        move_file(filepath, save_logs_dir)

        cpxfilepath = os.path.join(cpxlogsdir, file)
        move_file(cpxfilepath, save_cpxlogs_dir)

    runsum["stats"]["avg_ptime"] /= n_good
    runsum["stats"]["avg_stime"] /= n_good
    runsum["stats"]["avg_htime"] /= n_good
    runsum["stats"]["avg_btime"] /= n_good
    runsum["stats"]["avg_cbtime"] /= n_good
    runsum["stats"]["avg_ctime"] /= n_good
    runsum["stats"]["avg_time"] /= n_good
    runsum["stats"]["perc_found"] /= runsum["n_total"]
    runsum["stats"]["perc_opt"] /= runsum["n_total"]

    with open(f"{save_logs_dir}/001_{run_name}.json", "w") as f:
        json.dump(runsum, f, indent=4)


if __name__ == "__main__":
    main()
