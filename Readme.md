# HPLUS THESIS
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 2.2.2:5  
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
```

## Build options (CMake parameters)

- **-DCPLEX_PATH=**\<path_to_cplex>: (abs path), specify a custom path to the CPLEX installation (see the CMakeLists.txt to see which is the default one)
- **-DENABLE_WARNINGS=** ON: enable all types of compile warnings (default is OFF)

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
```
