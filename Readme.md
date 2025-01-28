# HPLUS THESIS
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Latest update: 28/01/25  

Using a MIP solver (CPLEX) to solve the delete free relaxation of a planning task.  

This software requires as input the SAS file produced by the [Fast Downward translator](https://www.fast-downward.org/).

## Index

- [Requirements](#requirements)
- [Build/Run](#buildrun-instructions)
- [Parameters](#parameters)
  - [CMake](#cmake-flags)
  - [Make](#make-flags)
  - [Execution](#run-options)
    - [Execution](#code-execution)
    - [Logging](#logging)

## Requirements

- UNIX based OS (didn't test on Windows if this works as it is)
- cmake 3.20 +

## Build/Run instructions

While inside the root folder of this repo:

```shell
# to build
mkdir code/build
cd code/build
cmake <build_options> ..
make <target_option>

# to run
./main <parameters>
```

## Parameters

### CMake flags

- **-DVERBOSE=**\<verbose_option> : set the verbose option (default: 5)
  - = **0** : no output
  - < **5** : just solution
  - \>= **5** : view time statistics
  - \>= **10** : basic steps progress
  - \>= **20** : debugging output
  - \>= **100** : full verbose
- **-DWARN=**\<warning_option> : set the warning option (default: 1)
  - **0** : all warnings suppressed
  - **1** : warnings will be showed
- **-DINTCHECK=**\<integrity_check_option> : set the integrity checks option (default: 0)
  - **0** : no integrity checks
  - **1** : integrity checks enabled (might slow down the execution)
- **-DCPLEX_DIR=**\<path_to_cplex_lib>: (abs path), specify a custom path to the CPLEX library (see the CMakeLists.txt to see which is the default one)
- **-DCPLEX_INCLUDE=**\<path_to_cplex_headers>: (abs path), specify a custom path to the CPLEX headers (see the CMakeLists.txt to see which is the default one)

### Make flags

- \<target_option> : specify the target build
  - _none_ : no flag added (default build)
  - **opt** : optimization flags added (optimized build)
  - **debug** : debugging flags added (debug build)
  - **clear** : removes all files inside the code/build folder

### Run options

#### Code execution

- **-i** <input_file> : (abs path) specify the input file
- **-a** <algorithm_name> : (string) (optional) specify the algorithm to use (default: rankooh)
  - **imai** : to use the model from the [Imai15 paper](references/Imai15.pdf)
  - **rankooh** : to use the model from the [Rankooh22 paper](references/Rankooh22.pdf)
  - **dynamic-s** : to use a dynamic model (small version) (preconditions & effects + dinamically added cuts)
  - **dynamic-l** : to use a dynamic model (bigger version) (preconditions & effects w. first archievers + dinamically added cuts)
  - **greedy** : to use a greedy algorithm (at each step, choose the action with the lowest cost per relevant variable)
- **-nos**: (optional) don't perform problem simplification
- **-notb**: (optional) don't use tighter timestamps bounds (only for imai)
- **-noheur**: (optional) don't look for an heuristic solution (this flag doesn't work for the greedy algorithm)
- **-nowarm**: (optional) don't use an heuristic solution as warm start (this flag doesn't work for the greedy algorithm)
- **-t** \<int> : (int) (optional) specify the time limit (in seconds) (default: 60)

#### Logging

- **-l** : (optional) tells the program to log output to file (as well as stdout)
- **-ln** <log_name> : (string/"string with spaces") (optional) specify the name of the log file (will be located inside the logs/AAA_output_logs folder); if not specified and the *-l* flag is used, a default log file will be used
- **-rn** <run_name> : (string/"string with spaces") (optional) set a run name that will show in the log file (used also to create log/lp files for cplex)
