import shutil
from pathlib import Path

current_dir = Path(__file__).parent
home_dir = current_dir.parent.parent.parent
build_dir = f"{home_dir}/code/build"
jobs_folder = current_dir.parent

logs_dir = f"{home_dir}/logs"
output_logs_dir = f"{logs_dir}/AAA_output_logs"
opt_logs_dir = f"{logs_dir}/AAB_opt_logs"
good_logs_dir = f"{logs_dir}/AAB_good_logs"
timelimit_logs_dir = f"{logs_dir}/AAC_timelimit_logs"
b_timelimit_logs_dir = f"{logs_dir}/AAD_b_timelimit_logs"
infease_logs_dir = f"{logs_dir}/AAE_infeas_logs"
errors_logs_dir = f"{logs_dir}/AAF_errors_logs"
other_logs_dir = f"{logs_dir}/AAF_other_logs"

def clear_dir(directory):

    for item in directory.iterdir():
        if item.is_file() or item.is_symlink():
            item.unlink()
        elif item.is_dir():
            shutil.rmtree(item)
