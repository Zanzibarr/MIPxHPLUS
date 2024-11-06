# ONLY FOR CLUSTER USE

from pathlib import Path
import utils

utils.clear_dir(Path(utils.output_logs_dir))
utils.clear_dir(Path(utils.opt_logs_dir))
utils.clear_dir(Path(utils.good_logs_dir))
utils.clear_dir(Path(utils.timelimit_logs_dir))
utils.clear_dir(Path(utils.b_timelimit_logs_dir))
utils.clear_dir(Path(utils.infease_logs_dir))
utils.clear_dir(Path(utils.errors_logs_dir))
utils.clear_dir(Path(utils.other_logs_dir))
for alg in ["imai", "rankooh"]:
    utils.clear_dir(Path(f"{utils.jobs_folder}/{alg}_jobs/job_outputs"))
