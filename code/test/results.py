import sys

sys.dont_write_bytecode

import subprocess, shlex, os, json, re
from pathlib import Path


def move_file(fromfilepath: Path, todir: Path):
    subprocess.run(shlex.split(f"mv {fromfilepath} {todir}/"))


def main():

    run_summary = {}

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

    logsdir = os.path.abspath("../../logs/output_logs")
    cpxlogsdir = os.path.abspath("../../logs/cpxout/log")
    save_logs_dir = Path(f"{Path(logsdir).parent}/saved_logs/{run_name}")
    save_cpxlogs_dir = Path(f"{Path(logsdir).parent}/saved_logs/{run_name}/cpxout")
    if os.path.exists(save_logs_dir):
        print(f"Path {save_logs_dir} already exists.")
        exit(1)

    print(f"Logs dir: {logsdir}")
    print(f"CPX logs dir: {cpxlogsdir}")
    print(f"Saved logs dir: {save_logs_dir}")
    print(f"Saved cpx logs dir: {save_cpxlogs_dir}")
    if input("Check those paths.\nInsert y if it's all correct: ") != "y":
        exit(0)

    run_summary["stats"] = {
        "avg_ptime": 0,
        "avg_stime": 0,
        "avg_htime": 0,
        "avg_btime": 0,
        "avg_rcbtime": 0,
        "avg_ccbtime": 0,
        "avg_ctime": 0,
        "avg_time": 0,
        "perc_found": 0,
        "perc_opt": 0,
    }
    run_summary["n_total"] = 0
    run_summary["other"] = ""
    run_summary["results"] = {}
    os.mkdir(save_logs_dir)
    os.mkdir(save_cpxlogs_dir)

    n_good = 0

    for file in os.listdir(logsdir):
        filepath = os.path.join(logsdir, file)
        with open(filepath, "r") as f:
            content = f.read()

        instance_name = Path(filepath).stem

        instance_data = {
            "status": -2,
            "unit_costs": False,
            "natoms": 0,
            "nacts": 0,
            "nfadd": 0,
            "nvar_base": 0,
            "nvar_acyc": 0,
            "nconst_base": 0,
            "nconst_acyc": 0,
            "usercuts_lm": 0,
            "usercuts_sec": 0,
            "nnodes": 0,
            "lb": 0,
            "hcost": 1e20,
            "fcost": 1e20,
            "incumb": [],
            "ptime": 0,
            "stime": 0,
            "htime": 0,
            "btime": 0,
            "rcbtime": 0,
            "ccbtime": 0,
            "ctime": 0,
            "time": 0,
            "other": "",
        }

        if "Axiom layer is" in content:
            instance_data["status"] = -3
            run_summary["results"][instance_name] = instance_data
            move_file(filepath, save_logs_dir)
            continue
        elif "[ ERROR ] OUT OF MEMORY" in content:
            instance_data["status"] = -1
            instance_data["other"] = "\n".join(content.splitlines()[-5:])
            run_summary["results"][instance_name] = instance_data
            run_summary["n_total"] += 1
            move_file(filepath, save_logs_dir)
            continue
        elif (
            "Version:" not in content
            or "[SUCCESS] Execution terminated" not in content
            or "[ ERROR ]" in content
        ):
            instance_data["other"] = "\n".join(content.splitlines()[-5:])
            run_summary["results"][instance_name] = instance_data
            run_summary["n_total"] += 1
            move_file(filepath, save_logs_dir)
            continue

        run_summary["n_total"] += 1

        params = content.partition(
            "------------------ List of parameters ------------------"
        )[2].partition("--------------------------------------------------------")[0]
        info = content.partition(
            "----------------- Info on the instance -----------------"
        )[2].partition("--------------------------------------------------------")[0]
        stats = content.partition(
            "---------------------- Statistics ----------------------"
        )[2].partition("--------------------------------------------------------")[0]

        if params == "" or info == "" or stats == "":
            instance_data["other"] = "\n".join(content.splitlines()[-5:])
            run_summary["results"][instance_name] = instance_data
            move_file(filepath, save_logs_dir)
            continue

        status = int(re.search(r">> Status\s+(\d+) <<", stats).group(1))
        if status == 4:
            instance_data["other"] = "\n".join(content.splitlines()[-5:])
            run_summary["results"][instance_name] = instance_data
            move_file(filepath, save_logs_dir)
            continue

        facts_prep = int(re.search(r">> Facts prep\s+(\d+) <<", stats).group(1))
        acts_prep = int(re.search(r">> Acts prep\s+(\d+) <<", stats).group(1))
        fadd_prep = int(re.search(r">> Fadd prep\s+(\d+) <<", stats).group(1))
        base_model_var = int(re.search(r">> Base model var\s+(\d+) <<", stats).group(1))
        acyc_model_var = int(re.search(r">> Acyc model var\s+(\d+) <<", stats).group(1))
        base_model_const = int(
            re.search(r">> Base model const\s+(\d+) <<", stats).group(1)
        )
        acyc_model_const = int(
            re.search(r">> Acyc model const\s+(\d+) <<", stats).group(1)
        )
        user_cuts_lm = int(re.search(r">> User cuts \(lm\)\s+(\d+) <<", stats).group(1))
        user_cuts_sec = int(
            re.search(r">> User cuts \(sec\)\s+(\d+) <<", stats).group(1)
        )
        nodes_expanded = int(re.search(r">> Nodes expanded\s+(\d+) <<", stats).group(1))
        lower_bound_str = re.search(
            r">> Lower bound\s+([\d.]+(?:e\d+)?) <<", stats
        ).group(1)
        lower_bound = float(lower_bound_str)
        heuristic_str = re.search(r">> Heuristic\s+([\d.]+(?:e\d+)?) <<", stats).group(
            1
        )
        if heuristic_str == "1e20":
            heuristic = 1e20
        else:
            heuristic = int(float(heuristic_str))
        final_cost_str = re.search(
            r">> Final cost\s+([\d.]+(?:e\d+)?) <<", stats
        ).group(1)
        if final_cost_str == "1e20":
            final_cost = 1e20
        else:
            final_cost = int(float(final_cost_str))
        if status == 0:
            lower_bound = float(final_cost)
        parsing_time = float(
            re.search(r">> Parsing time\s+([\d.]+)s <<", stats).group(1)
        )
        preprocessing_time = float(
            re.search(r">> Preprocessing time\s+([\d.]+)s <<", stats).group(1)
        )
        heuristic_time = float(
            re.search(r">> Heuristic time\s+([\d.]+)s <<", stats).group(1)
        )
        model_build_time = float(
            re.search(r">> Model build time\s+([\d.]+)s <<", stats).group(1)
        )
        relax_callback_time = float(
            re.search(r">> Relax callback time\s+([\d.]+)s <<", stats).group(1)
        )
        cand_callback_time = float(
            re.search(r">> Cand callback time\s+([\d.]+)s <<", stats).group(1)
        )
        cplex_time = float(re.search(r">> CPLEX time\s+([\d.]+)s <<", stats).group(1))
        total_time = float(re.search(r">> Total time\s+([\d.]+)s <<", stats).group(1))

        instance_data["status"] = status
        instance_data["unit_costs"] = "integer costs" not in info
        instance_data["natoms"] = facts_prep
        instance_data["nacts"] = acts_prep
        instance_data["nfadd"] = fadd_prep
        instance_data["nvar_base"] = base_model_var
        instance_data["nvar_acyc"] = acyc_model_var
        instance_data["nconst_base"] = base_model_const
        instance_data["nconst_acyc"] = acyc_model_const
        instance_data["usercuts_lm"] = user_cuts_lm
        instance_data["usercuts_sec"] = user_cuts_sec
        instance_data["nnodes"] = nodes_expanded
        instance_data["lb"] = lower_bound
        instance_data["hcost"] = heuristic
        instance_data["fcost"] = final_cost
        instance_data["ptime"] = parsing_time
        instance_data["stime"] = preprocessing_time
        instance_data["htime"] = heuristic_time
        instance_data["btime"] = model_build_time
        instance_data["rcbtime"] = relax_callback_time
        instance_data["ccbtime"] = cand_callback_time
        instance_data["ctime"] = cplex_time
        instance_data["time"] = total_time

        if status in (0, 1, 2):
            run_summary["stats"]["perc_found"] += 1
            if status in (0, 1):
                run_summary["stats"]["perc_opt"] += 1

        run_summary["stats"]["avg_ptime"] += parsing_time
        run_summary["stats"]["avg_stime"] += preprocessing_time
        run_summary["stats"]["avg_htime"] += heuristic_time
        run_summary["stats"]["avg_btime"] += model_build_time
        run_summary["stats"]["avg_rcbtime"] += relax_callback_time
        run_summary["stats"]["avg_ccbtime"] += cand_callback_time
        run_summary["stats"]["avg_ctime"] += cplex_time
        run_summary["stats"]["avg_time"] += total_time

        cpxlogfile = f"{cpxlogsdir}/{file.replace('.log', '.sas.log')}"
        if os.path.isfile(cpxlogfile):
            with open(cpxlogfile, "r") as f:
                cpxlogcontent = f.read()
            if instance_data["hcost"] != 1e20:
                instance_data["incumb"].append([float(0), instance_data["hcost"]])

            # Find all matches in the log content
            incumbent_pattern = re.compile(
                r"Found incumbent of value (\d+\.\d+) after (\d+\.\d+) sec\."
            )
            for match in incumbent_pattern.finditer(cpxlogcontent):
                # Extract value and time, convert to float
                value = float(match.group(1))
                time = float(match.group(2))

                # Append pair to the list
                instance_data["incumb"].append([time, value])

            move_file(cpxlogfile, save_cpxlogs_dir)

        run_summary["results"][instance_name] = instance_data
        move_file(filepath, save_logs_dir)
        n_good += 1

    run_summary["stats"]["avg_ptime"] /= n_good
    run_summary["stats"]["avg_stime"] /= n_good
    run_summary["stats"]["avg_htime"] /= n_good
    run_summary["stats"]["avg_btime"] /= n_good
    run_summary["stats"]["avg_rcbtime"] /= n_good
    run_summary["stats"]["avg_ccbtime"] /= n_good
    run_summary["stats"]["avg_ctime"] /= n_good
    run_summary["stats"]["avg_time"] /= n_good
    run_summary["stats"]["perc_found"] /= run_summary["n_total"]
    run_summary["stats"]["perc_opt"] /= run_summary["n_total"]

    run_summary["results"] = {
        key: run_summary["results"][key]
        for key in sorted(run_summary["results"].keys())
    }

    with open(f"{save_logs_dir}/002_{run_name}.json", "w") as f:
        json.dump(run_summary, f, indent=4)


if __name__ == "__main__":
    main()
