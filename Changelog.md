# Changelog

All notable changes to this project will be documented in this file.

_To see what I'm still working on, please refer to the [TodoList](Todolist.md)._  
_To see info about this project, please refer to the [Readme](Readme.md)._

<!--
HOW I DO VERSIONING:
- Increase x.. whenever I have a groundbreaking update to the code
- Increase .x. whenever a new feature has been properly tested
- Increase ..x for testing or developing new features / fixing bugs

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

## [1.0.3] - 2025/02/27
### :ballot_box_with_check: Fixed
- Logger formatting function didn't work properly
    - [~/code/external/log.hxx](code/external/log.hxx)
### :heavy_exclamation_mark: Changed
- Using std::stringstream to build string representations of pq and bs and formatting assertion error message for logger
    - [~/code/external/bs.hxx](code/external/bs.hxx)
    - [~/code/external/pq.hxx](code/external/pq.hxx)
    - [~/code/include/utils.hpp](code/include/utils.hpp)
### :heavy_plus_sign: Added
- hadd, hmax and relax heuristics
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :x: Removed -->
### :curly_loop: Other
- Little bit of code refactoring


## [1.0.2] - 2025/02/25
### :ballot_box_with_check: Fixed
- Errors not showing in log file
- Logs with errors appear in results json file with an empty string as name
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- Test scripts modify files or folders before the user confirmed correctness of all paths
    - [~/code/test/code/*](code/test/code)
- Update best solution didn't perform check on the solution if integrity checks are off (integrity of the solution should always be checked)
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
### :heavy_exclamation_mark: Changed
- Log output line is now monocromatic
    - [~/code/external/log.hxx](code/external/log.hxx)
- Modernized logger, binary_set and bs_searcher
    - [~/code/external/bs.hxx](code/external/bs.hxx)
    - [~/code/external/log.hxx](code/external/log.hxx)
### :heavy_plus_sign: Added
- Instances with errors in results json file now show last 5 lines of logs file (for better error understanding)
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
<!-- ### :x: Removed -->
### :curly_loop: Other
- Better readability in nested if and for loops


## [1.0.1] - 2025/02/24
### :ballot_box_with_check: Fixed
- Errors in paths int cluster jobs scripts
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
