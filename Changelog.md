# Changelog

All notable changes to this project will be documented in this file.

_To see info about this project, please refer to the [Readme](Readme.md)._

<!--
HOW I DO VERSIONING:
- Increase x.. whenever I have a groundbreaking update to the code
- Increase .x. whenever a new feature has been properly tested
- Increase ..x for testing or developing new features / fixing bugs

- Increasing a digit implies the reset of all following ones to 0 
-->

## Versions


<!-- ## [x.y.z] - YYYY/MM/DD -->
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.2.5] - 2025/03/25
### :warning: Known issues
- "Optimal" solution provided by the model with tb might not be optimal if problem optimization is being used (max_steps might be smaller than the number of fixed actions (0 cost actions))
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Saving cplex logs after tests on cluster
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- Handling out of memory CPLEX error
    - [~/code/include/utils.hpp](code/include/utils.hpp)
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
### :x: Removed
- Removed tighter bounds in Rankooh's and Dynamic models
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :curly_loop: Other -->

## [1.2.4] - 2025/03/21
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Infeasible warm start in Imai's model (sometimes)
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- Infeasible warm start in Rankooh's model (sometimes)
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- Infeasible warm start in dynamic model (sometimes)
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
### :heavy_exclamation_mark: Changed
- Hadd heuristic is now the default one
    - [~/code/src/main.cpp](code/src/main.cpp)
- Changed flag for tighter bounds option
    - [~/code/src/main.cpp](code/src/main.cpp)
### :heavy_plus_sign: Added
- Tight bounds on every model
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->

## [1.2.3] - 2025/03/20
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Faster problem simplification
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->

## [1.2.2] - 2025/03/19
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Imai's model won't validate warm starts for some instances
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Stricter integrity checks in computing heuristic solutions
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- Faster greedy-based heuristics
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.2.1] - 2025/03/18
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- better candidate actions lookup in greedy-based heuristics
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Trail to restore values after action simulation in hadd-based lookahead heuristic
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- Incremental candidate actions in greedy-based heuristics for faster feasible action lookup
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- CPLEX log showing every new incumbent
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
### :x: Removed
- Removed relax heuristic
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
    - [~/code/src/main.cpp](code/src/main.cpp)
### :curly_loop: Other
- Minor changes to the binary_set related classes
    - [~/code/external/bs.hxx](code/external/bs.hxx)

## [1.2.0] - 2025/03/17
### Tested all features added in 1.1.*
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Changed output logs folder name
    - [~/code/include/utils.hpp](code/include/utils.hpp)
    - [~/code/src/main.cpp](code/src/main.cpp)
    - [~/code/CMakeLists.txt](code/CMakeLists.txt)
    - [~/code/tst/code/results_jobs.py](code/test/code/results_jobs.py)
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.1.3] - 2025/03/14
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Removing radundant constraints from Rankooh's and Dynamic model
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- Showing number of fixed variables in high verbosity settings
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.1.2] - 2025/03/13
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Executable name changed from main to hplus
    - [~/code/CMakeLists.txt](code/CMakeLists.txt)
- Using pq.hxx insthead of std::priority_queue to build the vertex elimination graph
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->

## [1.1.1] - 2025/03/11
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Added dynamic time model
    - [~/code/src/algorithms.hpp](code/src/algorithms.hpp)
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.1.0] - 2025/03/11
### Tested all features added in 1.0.*
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Results reading script mis-labeling jobs halted on time limit even if a solution was found
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- Results reading script doesn't read final solution if it's not optimal
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- mypause method exiting on 0 instead of 1
    - [~/code/include/utils.hpp](code/include/utils.hpp)
### :heavy_exclamation_mark: Changed
- Better CMake file
    - [~/code/CMakeLists.txt](code/CMakeLists.txt)
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
### :curly_loop: Other
- Using google style as clang-format configuration
    - [~/.clang-format](.clang-format)


## [1.0.6] - 2025/03/04
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Input file, Run name and Log name prints in show_info() were poorly formatted
    - [~/code/src/main.cpp](code/src/main.cpp)
- Include errors for Linux
    - [~/code/include/utils.hpp](code/include/utils.hpp)
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
### :curly_loop: Other
- Started using clang-tidy as static code analyzer


## [1.0.5] - 2025/03/02
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Imai's model crashes if instance optimization is active
- Posting warm start to Imai's model might not find a feasible solution
- Posting warm start to Rankooh's model finds wrong objective
- Hmaxv1 and haddv1 heur find infeasible solutions even if they aren't
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
### :heavy_exclamation_mark: Changed
- Faster immediate action application with bs_searcher
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
- Better access to first adders in model building
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
- Using immutable objects whenever it's possible
### :heavy_plus_sign: Added
- Added integrity checks to find errors faster
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
- Heuristics now take into account fixed actions and fixed actions timestamps
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.0.4] - 2025/03/01
### :warning: Known issues
- Imai's model crashes if instance optimization is active
- Posting warm start to Imai's model might not find a feasible solution
- Posting warm start to Rankooh's model finds wrong objective
### :ballot_box_with_check: Fixed
- Script for reading results from logs only read the first heuristic solution found (in randr, we need to read the last one)
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- Removed unused imports
    - [~/code/*](code/)
- List of remaining variables and actions are calculated on each request, even if after optimization the return is always the same
    - [~/code/src/hplus_instance.cpp](code/src/hplus_instance.cpp)
### :heavy_exclamation_mark: Changed
- Better isint() function
    - [~/code/include/utils.hpp](code/include/utils.hpp)
- Instance optimization now removes deleted facts from all effects and precondition
- Changed hmax and hadd function
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
### :heavy_plus_sign: Added
- New version of hmax and hadd
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
<!-- ### :x: Removed -->
### :curly_loop: Other
- Code readability adjustments


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
