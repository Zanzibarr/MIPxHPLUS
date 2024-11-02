#!/bin/bash
#SBATCH --job-name=hplusImaiZanMatTEST
#SBATCH --partition=arrow
#SBATCH --ntasks=1
#SBATCH --mem=14GB
#SBATCH --time=00:10:00
# warm up processors
sudo cpupower frequency-set -g performance
sleep 0.1
stress-ng -c 4 --cpu-ops=100
# set limits
ulimit -v 16777216

#####################

export PYTHONPATH="$HOME/.notify2/python_module:$PYTHONPATH"
/usr/bin/python3 $HOME/thesis_hplus/code/testing/tester.py rankooh 5 /nfsd/rop/instances/PlanningSAS

#####################

# back to power saving mode
sudo cpupower frequency-set -g powersave