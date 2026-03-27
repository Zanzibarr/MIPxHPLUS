# MIP formulations for Deletefree AI Planning
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 3.1.3  
_Refer to the [Changelog](Changelog.md) for info about versions._  


## Description
This software solves the Delete-Free relaxation of automated planning tasks using Mixed Integer Programming (MIP) formulations. It reads planning problems in SAS+ format and computes optimal solutions using CPLEX.

## Index

- [Requirements](#requirements)
- [Build/Run](#buildrun-instructions)
- [CMake Parameters](#build-options-cmake-parameters)
- [Publications/Presentations](#publications-and-presentations)

## Requirements

- UNIX based OS
- cmake 3.20 +
- This software requires as input the SAS file produced by the [Fast Downward translator](https://www.fast-downward.org/latest/documentation/translator-output-format/) (this software was produced with version 3 of the FD translator as a reference)

## Build/Run instructions

While inside the root folder of this repo:

```shell
# to build
mkdir code/build
cd code/build
cmake <build_options> ..
make

# to run the code on an instance
./hplus <input_file> <parameters>
```

### Build options (CMake parameters)

- **-DCPLEX_PATH=**\<path_to_cplex>: (absolute path), specify a custom path to the CPLEX installation (see the CMakeLists.txt to see which are the default ones (CPLEX_POSSIBLE_PATHS) or append your to that list to avoid specifying it every time)
- **-DCMAKE_BUILD_TYPE=**Debug/Release: Build type (default is Debug), Debug mode enables warnings

### Run options

See the help page:
```shell
# to view commands available
./hplus --h
```

Here's a few examples:
```shell
# Run the vertex elimination algorithm with a time limit of 5 min
./hplus problem.sas --a=ve --t=300

# Run the greedy algorithm with the hadd lookahead strategy and no preprocessing
./hplus problem.sas --a=gha --prep=0

# Run the Landmark based model with various customizations
./hplus problem.sas --a=cuts \
    --ws=0 \          # Don't use the primal heuristic as warm start
    --fract-minlm=0 \ # Don't use the LM minimalization procedure in relax callback
    --cloop=0         # Don't use custom cutloop
```

## Publications and Presentations
- **Master Thesis** (2025): [\[Thesis\]](results/2025_master_thesis.pdf)  
- **AIROYoung Workshop** (2026): [\[Slides\]](results/2026_AIROYoung_slides.pdf)  
- **IJOO MIP Workshop** (2025, under review): [\[Preprint\]](results/2025_MIPWorkshopIJOO_preprint.pdf)

## License
MIT License - see [LICENSE](LICENSE) for details.