# Changelog

All notable changes to this project will be documented in this file.

_To see what I'm still working on, please refer to the [TodoList](Todolist.md)._  
_To see info about this project, please refer to the [Readme](Readme.md)._

<!--
HOW I DO VERSIONING:
- Increase x.. whenever I have a groundbreaking update to the code
- Increase .x. whenever I have effective user experience changes or important "invisible" changes to the code execution (es major optimization of a method)
- Increase ..x whenever I change somthing that won't affect much user experience (small visual changes on the output fall in this category)

- Increasing a digit implies the reset of all following ones to 0 
-->

## Versions

<!-- 
## [x.y.z] - YYYY/MM/DD
### :ballot_box_with_check: Fixed
### :heavy_exclamation_mark: Changed
### :heavy_plus_sign: Added
### :x: Removed
### :curly_loop: Other
-->

## [1.0.1] - 2025/02/24
### :ballot_box_with_check: Fixed
- Errors in cluster jobs scripts
    - [~/code/test/code/*](code/test/code/)
### :heavy_exclamation_mark: Changed
- Using return value of std::set::insert to check if element is actually added to a set
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
### :heavy_plus_sign: Added
- Documentation for binary_set, bs_searcher and logger classes for future reuse
    - [~/code/external/bs.hxx](code/external/bs.hxx)
    - [~/code/external/log.hxx](code/external/log.hxx)
### :x: Removed
- Unused private function in logger
    - [~/code/external/log.hxx](code/external/log.hxx)
### :curly_loop: Other
- Automatic code formatting with clang-format (cpp) and black-formatter (python)
- Start using attributes ([[nodiscard]], [[likely]], [[unlikely]])

## [1.0.0] - 2025/02/23

### :ballot_box_with_check: Fixed
- Help function in test scripts is now resistant to scripts name changing
    - [~/code/test/code/*](code/test/code/)
- Added time-limit control inside heuristic algorithms
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Versioning
- Script to check correctness of a batch run of instances
    - [~/code/test/code/check_costs.py](code/test/code/check_costs.py)
- Help option to the results reading script
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- Added detection of instances with constant cost actions
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
    - _other minor changes_
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
