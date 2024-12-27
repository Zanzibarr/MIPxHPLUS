# ONLY FOR CLUSTER USE

import sys
sys.dont_write_bytecode = True

from pathlib import Path
import utils

utils.clear_dir(Path(utils.output_logs_dir))
utils.clear_dir(Path(utils.opt_logs_dir))
utils.clear_dir(Path(utils.feas_logs_dir))
utils.clear_dir(Path(utils.timelimit_logs_dir))
utils.clear_dir(Path(utils.b_timelimit_logs_dir))
utils.clear_dir(Path(utils.infease_logs_dir))
utils.clear_dir(Path(utils.errors_logs_dir))
utils.clear_dir(Path(utils.other_logs_dir))
for alg in ["imai", "rankooh", "dynamic", "greedy"]:
    utils.clear_dir(Path(f"{utils.jobs_folder}/{alg}_jobs/job_outputs"))
