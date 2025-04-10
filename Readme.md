# HPLUS THESIS
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 1.3.4  
_Refer to the [Changelog](Changelog.md) for info about versions._  


Using a MIP solver (CPLEX) to solve the (optimal) delete free relaxation of a planning task.  

This software requires as input the SAS file produced by the [Fast Downward translator](https://www.fast-downward.org/).

## Index

- [Requirements](#requirements)
- [Build/Run](#buildrun-instructions)
- [Parameters](#parameters)
  - [CMake](#cmake-flags)
  - [Make](#make-flags)
  - [Execution](#run-options)

## Requirements

- UNIX based OS
- cmake 3.20 +

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
# to view info about an instance
./info <input_file>
```

## Build options (CMake parameters)

- **-DVERBOSE=**\<verbose_option> : set the verbose option (default: 5)
  - = **0** : no output
  - < **5** : just solution
  - \>= **5** : view statistics
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

## Target options (Make parameters)

- \<target_option> : specify the target build
  - _none_ : no flag added (default build)
  - **opt** : optimization flags added (optimized build)
  - **debug** : debugging flags added (debug build)
  - **clear** : removes all files inside the code/build folder

## Run options

See the help page:
```shell
# to view commands available
./hplus --h
./info --h
```
