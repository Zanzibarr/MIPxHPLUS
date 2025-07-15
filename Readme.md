# MIP formulations for Deletefree AI Planning
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 2.2.3  
_Refer to the [Changelog](Changelog.md) for info about versions._  


## Abstract
We compare current state-of-the-art M.I.P. formulations for solving the delete-free relaxation in cost-optimal planning, integrating various preprocessing techniques from different publications and developing primal heuristics to provide warm-start solutions to the M.I.P. solver.  
We then explore a novel approach to model acyclicity by computing violated landmarks (aswell as S.E.C.) and adding them as constraints: computing all landmarks a priori is intractable, since the number of potential landmarks grows exponentially with the problem size, just like S.E.C., making any brute-force approach computationally infeasible; instead we employ a constraint generation approach where new constraints are identified dynamically: upon encountering an infeasible solution, we detect violated cuts within that solution and incorporate them as additional constraints, progressively constructing the minimal constraint set required for both feasibility and optimality.  
Our experimental evaluation demonstrates that this landmark-based formulation achieves competitive performance with existing methods in both space and time efficiency.

## Publication associated to this code
_Work in progress_

## Index

- [Requirements](#requirements)
- [Build/Run](#buildrun-instructions)
- [Parameters](#build-options-cmake-parameters)
  - [CMake](#build-options-cmake-parameters)
  - [Make](#target-options-make-parameters)
  - [Execution](#run-options)

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

## Build options (CMake parameters)

- **-DCPLEX_PATH=**\<path_to_cplex>: (absolute path), specify a custom path to the CPLEX installation (see the CMakeLists.txt to see which are the default ones (CPLEX_POSSIBLE_PATHS) or append your to that list to avoid specifying it every time)
- **-DENABLE_WARNINGS=** 1: enable all types of compile warnings (default is 0)

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
