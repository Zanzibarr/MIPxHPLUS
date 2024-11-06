import sys
sys.dont_write_bytecode = True
import utils, os

times = []

for file in os.listdir(utils.opt_logs_dir):
    with open(f"{utils.opt_logs_dir}/{file}", "r") as f:
        content = f.read()
    
    parsing_time = float(content.partition(">>  Parsing time")[2].partition("s")[0].strip())
    opt_time = float(content.partition(">>  Optimization time")[2].partition("s")[0].strip())
    wst_time = float(content.partition(">>  Warm-start time")[2].partition("s")[0].strip())
    build_time = float(content.partition(">>  Build time")[2].partition("s")[0].strip())
    exec_time = float(content.partition(">>  Exec time")[2].partition("s")[0].strip())

    cost = int(content.partition("Solution cost:")[2].partition("\n")[0].strip())
    total_time = parsing_time + opt_time + wst_time + build_time + exec_time
    
    instance_name = file.replace("_imai.log", "").replace("_rankooh.log", "")

    if not os.path.exists(f"{utils.home_dir}/fast_downward_baseline/{instance_name}"): continue

    with open(f"{utils.home_dir}/fast_downward_baseline/{instance_name}", "r") as f:
        content_baseline = f.read().splitlines()
    
    cost_range = [float(x) for x in content_baseline[0].split(" ")]
    time = float(content_baseline[1])

    if cost < cost_range[0] or cost > cost_range[1]: print(f"ERROR: {instance_name}")

    times.append([time, total_time])

input()

[print(x) for x in times]