# MIP formulations for Deletefree AI Planning
![C++](https://img.shields.io/badge/C%2B%2B-00599C?style=for-the-badge&logo=c%2B%2B&logoColor=white)

### Version: 4.1.2  
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
- IBM ILOG CPLEX Optimization Studio

## Build/Run instructions

While inside the root folder of this repo:

```shell
# to build
mkdir code/build
cd code/build
cmake <build_options> ..
make

# to run the code on an instance
./hplus -i <input_file> <parameters>
```

### Build options (CMake parameters)

- **-DCMAKE_BUILD_TYPE=**\<Debug/Release>: Build type (default is Debug), Debug mode enables warnings
- **-DENABLE_ASSERTS=**\<ON/OFF>: Specify whether to enable asserts in non-Debug builds (default is OFF)
- **-DCPLEX_PATH=**\<path_to_cplex>: (absolute path), specify a custom path to the CPLEX installation (see the CMakeLists.txt to see which are the default ones (CPLEX_POSSIBLE_PATHS) or append your to that list to avoid specifying it every time)

### Run options

See the help page:
```shell
# to view commands available
./hplus -h
```

## Publications and Presentations
- **Master Thesis** (2025, v2.4.2): [\[Thesis\]](results/2025_master_thesis.pdf)  
- **AIROYoung Workshop** (2026, v2.4.2): [\[Slides\]](results/2026_AIROYoung_slides.pdf)  
- **IJOO MIP Workshop** (2025, v2.4.2, under review): [\[Preprint\]](results/2025_MIPWorkshopIJOO_preprint.pdf)
- **CPAIOR Extended Abstract** (2026, v3.1.10): [\[Slides\]](results/2026_CPAIOR_slides.pdf)  
- **IPCO Poster** (2026, v3.1.10): [\[Poster\]](results/2026_IPCO_poster.pdf)  

## License
MIT License - see [LICENSE](LICENSE) for details.
