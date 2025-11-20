# MIP formulations for Deletefree AI Planning
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 2.4.1  
_Refer to the [Changelog](Changelog.md) for info about versions._  


## Abstract
We investigate existing Mixed Integer Programming (MIP) formulations for cost-optimal delete-free STRIPS Planning:
these models are built by enforcing acyclicity in the underlying causal relation graphs using time labeling and vertex elimination
methods. We then propose two new approaches to modeling acyclicity, one based on dynamically identifying subtour elimination
constraints, and the other based on disjunctive landmark constraints. In addition, we propose to warm start the models with the
landmarks computed by the LM-cut heuristic, and describe a simple greedy primal heuristic to provide a starting feasible solution
to the MIP solver. Our results demonstrate that the proposed techniques outperform the current state of the art, both in space and
time efficiency.

## Publications associated to this code
Master thesis in Computer Engineering: [\[pdf\]](results/MIP_formulations_for_deletefree_AI_planning.pdf)  
IJOO MIP Workshop 2025: _Work in progress_

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
