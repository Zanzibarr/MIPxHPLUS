# HPLUS THESIS

Using a MIP solver (CPLEX) to solve the delete free relaxation of a planning task.

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

- **-DVERBOSE=**\<verbose_option>:
  - = **0** : no output
  - \> **0** : basic output
  - \>= **10** (or _none_) : basic steps progress
  - \>= **100** : full verbose
- **-DWARN=**\<warning_option>:
  - **0** : all warnings suppressed
  - **1** (or _none_) : warnings will be showed
- **-DINTCHECK=**\<integrity_check_option>:
  - **0** : no integrity checks
  - **1** (or _none_) : integrity checks enabled (might slow down the execution)
- **-DCPLEX_DIR=**\<path_to_cplex_lib>: optional, specify a custom path to the CPLEX library
- **-DCPLEX_INCLUDE=**\<path_to_cplex_headers>: optional, specify a custom path to the CPLEX headers

### Make flags

- \<target_option>:
  - _none_ : no flag added (default build)
  - **opt** : -O3 flag added (optimized build)
  - **debug** : -g flag added (debug build)
  - **clear** : removes all files inside the code/build folder

### Run options

#### Code execution

- **-i** <input_file> : specify the input file (looked up inside the code/instances folder)
- **-a** <algorithm_name> : specify the algorithm to use
  - **imai** to use the baseline model from the Imai15 paper

#### Logging

- **-l** : (optional) tells the program to log output to file (as well as stdout)
- **-ln** : (optional) specify the name of the log file (will be located inside the code/logs folder); if not specified and the _-l_ flag is used, a default log file will be used
- **-rn** : (optional) set a run name that will show in the log file (used also to create log/lp files for cplex)
