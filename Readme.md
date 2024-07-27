# HPLUS THESIS

Using a MIP solver (CPLEX) to solve the delete free relaxation of a planning task.

## Index
- [Requirements](#requirements)
- [Build/Run](#buildrun-instructions)
- [Parameters](#parameters)

## Requirements

- UNIX based OS (didn't test on Windows if this works as it is)
- cmake 3.20 +

## Build/Run instructions

While inside the root folder of this repo:

```shell
# to build
mkdir code/build
cd code/build
cmake -D VERBOSE=<verbose_option> -D WARN=<warning_option> -D INTCHECK=<integrity_check_option> ..
make <target_option>

# to run
./main <parameters>
```

## Parameters

### Build options

- verbose_option:
  - == **0** : no output
  - \> **0** : basic output
  - \>= **10** (or _none_) : basic steps progress
  - \>= **100** : full verbose
- warning_option:
  - **0** : all warnings suppressed
  - **1** (or _none_) : warnings will be showed
- integrity_check_option:

  - **0** : no integrity checks
  - **1** (or _none_) : integrity checks enabled (might slow down the execution)

- target_option:
  - _none_ : no flag added (default build)
  - **opt** : -O3 flag added (optimized build)
  - **debug** : -g flag added (debug build)

### Run options

#### Code execution

- **-i** <input_file> : specify the input file (looked up inside the code/instances folder)

#### Logging

- **-l** : (optional) tells the program to log output to file (as well as stdout)
- **-ln** : (optional) specify the name of the log file (will be located inside the code/logs folder); if not specified and the _-l_ flag is used, a default log file will be used
- **-rn** : (optional) set a run name that will show in the log file
