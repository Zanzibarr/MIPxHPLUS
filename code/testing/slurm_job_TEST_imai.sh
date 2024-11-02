#!/bin/bash
#SBATCH --job-name=hplusImaiZanMatTEST
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

cd "$(dirname "$0")"
export PYTHONPATH="$HOME/.notify2/python_module:$PYTHONPATH"
/usr/bin/python3 tester.py imai 5 /nfsd/rop/instances/PlanningSAS

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave