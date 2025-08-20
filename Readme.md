# MIP formulations for Deletefree AI Planning
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 2.2.6:2  
_Refer to the [Changelog](Changelog.md) for info about versions._  


## Abstract
We compare current state-of-the-art MIP formulations for solving the delete-free relaxation in cost-optimal planning, integrating various preprocessing techniques from different publications and developing primal heuristics to provide warm-start solutions to the MIP solver. We then explore a novel approach to model acyclicity by computing violated landmarks (as well as S.E.C.) and adding them as constraints: computing all landmarks a priori is intractable, since the number of potential landmarks grows exponentially with the problem size, just like S.E.C., making any brute-force approach computationally infeasible; instead we employ a constraint generation approach where new constraints are identified dynamically: upon encountering an infeasible solution, we detect violated cuts within that solution and incorporate them as additional constraints, progressively constructing the minimal constraint set required for both feasibility and optimality. Our experimental evaluation demonstrates that this landmark-based formulation achieves competitive performance with existing methods in both space and time efficiency.

## Publication associated to this code
_Work in progress_

## Index

- [Requirements](#requirements)
- [Build/Run](#buildrun-instructions)
- [Parameters](#build-options-cmake-parameters)
  - [CMake](#build-options-cmake-parameters)
  - [Make](#target-options-make-parameters)
  - [Execution](#run-options)
- [Testing](#testing)

## Requirements

- UNIX based OS
- cmake 3.20 +
- This software requires as input the SAS file produced by the [Fast Downward translator](https://www.fast-downward.org/latest/documentation/translator-output-format/) (this software was produced with version 3 of the FDW translator as a reference)

## Build/Run instructions

While inside the root folder of this repo:

```shell
# to build
mkdir code/build
cd code/build
cmake <build_options> ..
make <target_option>

# to run the code on an instance
./hplus <input_file> <parameters>
```

### Build options (CMake parameters)

- **-DCPLEX_PATH=**\<path_to_cplex>: (absolute path), specify a custom path to the CPLEX installation (see the CMakeLists.txt to see which are the default ones (CPLEX_POSSIBLE_PATHS) or append your to that list to avoid specifying it every time)
- **-DENABLE_WARNINGS=** 1: enable all types of compile warnings (default is 0)

### Target options (Make parameters)

- \<target_option> : specify the target build
  - _none_ : no flag added (default build)
  - **opt** : optimization flags added (optimized build)
  - **debug** : debugging flags added (debug build)
  - **clear** : removes all files inside the code/build folder

### Run options

See the help page:
```shell
# to view commands available
./hplus --h
```

## Testing
All python3 scripts used for testing are available in the [test](code/test/) folder.
The scripts with this line as the first line:
```python
# ONLY FOR CLUSTER USE
```
are meant to be used inside UNIPD's cluster for testing, hence those will not work out of the box in another machine/environment.

Testing is ment to be working as follows:

1) Create the jobs with the [create.py](code/test/create.py) script -- this will create batches of 1000 jobs each, located inside the code/test/jobs/ folder (that will be created upon running this script)
```bash
# this will create the folder code/test/jobs, where the batches will be created, each in a folder batch_[idx], and code/test/jobs_output where all jobs stdout/stderr will be located (note that the hplus program output will be located in logs/output_logs/ instead)
python3 create.py <path_to_instances> [<execution>, <parameters>]
```
2) Run the jobs one batch at the time with the [run.py](code/test/run.py) script, specifying the index of the batch to be run (if 4 batches have been created, the indexes will be [0,1,2,3])
```bash
# this will run the jobs in the folder jobs/batch_0/
python3 run.py 0
```
3) As stated before, execution logs will be stored in the logs/output_logs folder: from now on, whichever was the way the jobs were executed, if the execution of all instances to be tested have a log inside the logs/output_logs folder, you can use the [results.py](code/test/results.py) script
```bash
# this will create a json file containing all usefull info about all the run logs stored in the logs/output_logs folder
# all the logs will be stored inside logs/saved_logs/<run_name>, aswell as the json summary, saved as 002_<run_name>.json inside the same folder
python3 results.py <run_name>
```
4) Once you have enough json files, you can perform analysis on them, using the following scripts:
- [comparison.py](code/test/comparison.py)
- [comparison_plots.py](code/test/comparison_plots.py)
- [show_times.py](code/test/show_times.py)

### Dependencies
For this scripts to run you will need to have those python modules installed:
```
numpy
matplotlib
scipy
```
You can easily install them only for this project by creating a virtual environment:
```bash
# creating the virtual environment and installing dependencies
python3 -m venv testing_environment
source testing_environment/bin/activate
pip install numpy matplotlib scipy
```
_Remember to enter the environment with the "source ..." command each time you want to run those scripts on a new terminal_

### How to use these scripts
```bash
# to perform an 1-1 comparison between a specific metric on two runs (e.g. measure the number of nodes cplex expanded)
# this script divides the instances based on the lowest time among the two runs for that instance
python3 comparison.py <metric> <run_1> <run_2>
# to plot the basic comparisons between two runs -- the plot will be saved at code/test/plots/comparison.svg
python3 comparison_plots.py <run_1> <run_2>
# to view the number of instances solved within a certain time (cumulative plot) -- the plot will be saved at code/test/plots/times.svg
python3 show_times.py <run_1> ...
```