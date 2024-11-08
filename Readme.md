# HPLUS THESIS
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

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

- **-DVERBOSE=**\<verbose_option> : set the verbose option (default: 1)
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
- **-a** <algorithm_name> : (string) specify the algorithm to use
  - **imai** : to use the model from the [Imai15 paper](references/Imai15.pdf)
  - **rankooh** : to use the model from the [Rankooh22 paper](references/Rankooh22.pdf)
- **-opt-enhance** <0/1>: (bool) (optional) specify wether to use or not model enhancements algorithms (default: 1)
  - **0** : to disable this option
  - **1** : to enable this option
- **-opt-heur-1** <0/1>: (bool) (optional) specify wether to search for an heuristic solution (probably a bad one, but at least a solution is almost always found) (default: 1)
  - **0** : to disable this option
  - **1** : to enable this option
- **-opt-heur-2** <0/1>: (bool) (optional) specify wether to search for an optimized heuristic solution (better than the one found with -opt-heur-1) (default: 1)
  - **0** : to disable this option
  - **1** : to enable this option
- **-opt-warmstart** <0/1>: (bool) (optional) specify wether to use or not an heuristic solution as warm start (default: 1)
  - **0** : to disable this option
  - **1** : to enable this option
- **-opt-imai-var-bound** <0/1>: (bool) (optional) specify wether to use or not tighter bounds on the timestamp variables (to use with the imai algorithm) (default: 1)
  - **0** : to disable this option
  - **1** : to enable this option
- **-t** \<int> : (int) (optional) specify the time limit (in seconds) (default: 60)

#### Logging

- **-l** : (optional) tells the program to log output to file (as well as stdout)
- **-ln** <log_name> : (string/"string with spaces") (optional) specify the name of the log file (will be located inside the logs/AAA_output_logs folder); if not specified and the *-l* flag is used, a default log file will be used
- **-rn** <run_name> : (string/"string with spaces") (optional) set a run name that will show in the log file (used also to create log/lp files for cplex)
