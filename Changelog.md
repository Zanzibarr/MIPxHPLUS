# Changelog

All notable changes to this project will be documented in this file.

_To see info about this project, please refer to the [Readme](Readme.md)._

<!--
HOW I DO VERSIONING:
- Increase x... for a groundbreaking update to the code (e.g. refactoring of some kind)
- Increase .x.. for updated/new feature (that change what the code does, eg different algorithm)
- Increase ..x. for new implementations (that doesn't change fundamentaly what the code does)
- Increase ...x for bug fixes

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
<!-- ### :rocket: Performance Improvements -->


## [3.1.7] - 2026/04/28
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- [3.1.7.1] Uncaught exception in lmcut computation
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Added option to add fractional callbacks to complete (TL and VE) models (aswell as custom cutloop)
- [3.1.7.2] Added integrity check in reading CPLEX solution
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [3.1.6] - 2026/04/09
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- [3.1.6.1] Patch applied to candidate callback: sometimes CPLEX takes too much to realize that a posted solution is the optimal one... if we figure it out before it, we terminate the execution
- [3.1.6.2] Patch applied to vertex elimination model: warm start posted was infeasible (unchecked, so still accepted) due to a wrong construction of vertex elimination graph variables
### :heavy_exclamation_mark: Changed
- Updated defaults for new best model
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- lmcC_flmcG_lmcC_hadd with vs without cutloop (3.1.5.1):
    - Solved: 2646 -> 2646
    - Nodes: +19,-34,-38,-44%
    - Time: -5,-10,-0,+4%
- lmcC_lmcC_hadd vs lmcC_flmcG_lmcC_hadd with no cutloop (3.1.5.1):
    - Solved: 2619 -> 2648
    - Nodes: -33,-72,-80,-97%
    - Time: -0,-1,-10,-40%
- New best model is lmcC_flmcG_lmcC_hadd with no cutloop (default values have been updated)


## [3.1.5] - 2026/03/29
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- [3.1.5.1] Missing epsilon in fractional landmark violation check
- [3.1.5.1] Using CPX_USECUT_FILTER in relaxation callback
- [3.1.5.2] Wrong imports in cutloop functions
- [3.1.5.2] Missing error checks in cutloop functions
### :heavy_exclamation_mark: Changed
- Updated greedy implementation
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- 3.1.4.1 vs 3.1.5 on compC_lmcutC_hadd:
    - Heuristic Time: -39,-27,-21,-28%
    - Solved: 2566 -> 2566
    - Nodes: -0, -0, -0, -0%
    - Time: -2,-4,-5,-2%


## [3.1.4] - 2026/03/28
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- [3.1.4.1] Timelimit implementation slowed down fast instances
### :heavy_exclamation_mark: Changed
- Optimized parsing implementation
- Optimized preprocessing implementation
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- 3.1.3 vs 3.1.4.1 on compC_lmcutC_hadd:
    - File Parsing Time: -18,-20,-18,-24%
    - Preprocessing Time: -33,-28,-17,-11%
    - Solved: 2566 -> 2566
    - Nodes: -0, -0, -0, -0%
    - Time: -17,-16,-4,-0%


## [3.1.3] - 2026/03/27
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Updated greedy implementation
- Updated hmax/hadd greedy choice implementation
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [3.1.2] - 2026/03/26
### :warning: Known issues
- Execution on MacOS is (sometimes) non-deterministic (this is a CPLEX bug for ARM MacOS)
### :ballot_box_with_check: Fixed
- Wrong time measurements mess up results for fastest runs
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [3.1.1] - 2026/03/21
### :warning: Known issues
- Execution on MacOS is (sometimes) non-deterministic
### :ballot_box_with_check: Fixed
- When int separator based on LMcut didn't find any solution, the callback didn't fallback to exact method
- Logger in async mode doesn't print error or success messages being too close to code termination
- Un-handled CPLEX solution status (CPXMIP_FEASIBLE)
### :heavy_exclamation_mark: Changed
- Updated version of logger for bugfixes
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [3.1.0] - 2026/03/20
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Minor bug fixes and dead/no-op code removed 
### :heavy_exclamation_mark: Changed
- Using watch-preconditions (inspired from watch literals in sat solvers) for reachability analysis in landmark minimalization
### :heavy_plus_sign: Added
- Added TODOs on possible future improvements
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- Implementation improvements on landmark minimalization algorithm
- Old LMC (just LMcut on integer solutions with minimalization (both greedy and extensive)) vs with new minimalization
    - Solved: 2624 -> 2625
    - Nodes: -0, -0, -0, -0%
    - Time: -4,-0,-5,-4%


## [3.0.0] - 2026/03/19
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Removed suboptimal use of BinarySet
- Updated BinarySet, Logger implementations
### :heavy_plus_sign: Added
- Using StatsRegistry for timing and statistics
- Landmark minimization in LMcut
### :x: Removed
- BinarySet pre and eff from actions
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- Landmark minimization in LMcut (compared with test run):
    - Solved: 2564 -> 2594
    - Nodes: -11,-50,-57,-38%
    - Time: +57,-30,-40,-24%
- Implementation adjustments (compared with LM min):
    - Solved: 2594 -> 2600
    - Nodes: -3,-0,-5,-12%
    - Time: -31,-21,-33,-14%
- Old baselines are now outdated... need to run new benchmarks to find the best model


## [2.6.1] - 2026/03/10
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Fixed precision errors for close-to-0 values in fractional LMcut separator
### :heavy_exclamation_mark: Changed
- Smarter landmark selection in LMcut
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [2.6.0] - 2026/03/05
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Additional init() removed 0ing of used actions in lmcut int separator
### :heavy_exclamation_mark: Changed
- Updated LMcut implementation
### :heavy_plus_sign: Added
- Precise time measuring for LMcut in preprocessing
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- Unchanged performances


## [2.5.1] - 2026/02/18
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Option to use LMcut to separate violated landmark constraints from integer solutions
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- Violated landmark constraints can now be separated using the LMcut algorithm, by setting the reduced costs of used actions to 0 (note that this separation procedure is not complete when there are 0-cost actions, so it will be complemented by the complementary landmark separation procedure)
- Old LMC (just comp) vs new LMC (comp + lmc, no separation on fractional solutions):
    - Solved: 2520 -> 2582
    - Nodes: -58,-3,-56,-17%
    - Time: -36,+15,-41,-28%
- Best (FLM with minimalization) vs new LMC (comp + lmc, no separation on fractional solutions):
    - Solved: 2559 -> 2582
    - Nodes: -32,+45,+5,+75%
    - Time: -20,-7,-26,-32%
- Our Best method now uses both "comp" and "lmc" separation procedures on integer solutions, and doesn't separate landmarks on fractional solutions


## [2.5.0] - 2026/02/16
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Couldn't properly set the build type
- Lower Bound after cutloop didn't get updated correctly when reaching time limit
- Wrong vector sizes in basic model construction
### :heavy_exclamation_mark: Changed
- Updated list of best known solutions (both structure and added new values)
### :heavy_plus_sign: Added
- Minimalization of (violated) landmark found in relax callback
- Cutoff CLI parameter to be passed to CPLEX as upper cutoff
- Statistic about lower bound before starting the cutloop
- Statistics about LM size in fractional
- Compile time in executable output
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- Violated landmarks from fractional solutions are now improved through a local search strategy (minimalization of the landmark)
- Old vs new FLM (Fractional LandMark):
    - Solved: 2524 -> 2559
    - Nodes: -36,-27,-36,-14%
    - Time: -6,-11,-13,-23%
- Best vs new FLM:
    - Solved: 2520 -> 2559
    - Nodes: -45,-35,-49,-65%
    - Time: -2,+2,-16,-10%
- Our Best method now uses FLM with minimalization


## [2.4.2] - 2025/11/24
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Bug in fractional SEC where we didn't select the smallest-weight edge in the graph construction
### :heavy_exclamation_mark: Changed
- Minor aestetic changes in output
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [2.4.1] - 2025/11/20
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- LM-cut CLI parameter is now a string, so that it's possible to choose how many times to repeat LM-cut and with which pcf each time
### :heavy_plus_sign: Added
- Added missing warnings in CLI parsing
### :x: Removed
- Removed unused cutval calculation in landmark fract separation
<!-- ### :curly_loop: Other -->
<!-- ### :rocket: Performance Improvements -->


## [2.4.0] - 2025/11/19
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Little bug with cutloop pruning of non-tight constraints
### :heavy_exclamation_mark: Changed
- Minor code aestetics changes
- Fractional Landmark Separator is now an heuristic: choice of a PCF (thus, an heuristic) and max-flow algorithm to detect violated landmark
### :heavy_plus_sign: Added
- Added random seed choice for reproducibility
- Added LM-cut flag to toggle the use of LM-cut in preprocessing phase
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->
### :rocket: Performance Improvements
- Fractional Landmarks Separator is now an heuristic with a 95+% accuracy (before it was an exact algorithm, looking for the maximal violated landmark)
- Old vs new FLS:
    - Solved: 2516 -> 2522
    - Nodes: +26,+27,+69,+50%
    - Time: -16,-7,-7,-20%
- Best vs new FLS:
    - Solved: 2518 -> 2522
    - Nodes: -13,-12,-25,-51%
    - Time: +9,+9,+1,+16%


## [2.3.0] - 2025/10/15
### Tested all features added in 2.2.*
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Removed frontier landmarks from default option (candidate callback)
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.12] - 2025/10/14
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Issue with SEC cuts of fractional solution... it was generating cuts that weren't violated (a missing -1 basically T^T)
### :heavy_exclamation_mark: Changed
- Testing sorting of neighbor list for better cycle detection
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.11] - 2025/10/11
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Optimizations and other options in candidate callback for SEC cycle detection
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.10] - 2025/10/09
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Optimizations in candidate callback for complementary landmarks
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.9] - 2025/10/01
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Bug with choice of fractional cuts to use, both in the cutloop or in CPLEX callbacks
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Option to remove fractional cuts at nodes
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.8] - 2025/09/25
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Relaxation callback now executes at most once per node, except for nodes with depth 0 (root nodes, either at the start or after a restart)
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.7] - 2025/09/16
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Fixed issue with landmark generated from the relaxed solutions
- Bug where lower bound for the cutloop didn't get updated for timelimit termination
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Added insightful comments throughout the codebase
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.6] - 2025/08/22
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Fixed bug with LMCUT in version 2.2.6:4 \[:5\] (25/08/21)
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- LM-CUT disjunctive action landmarks for preprocessing (25/08/17)
- Testing random LMCUT preprocessing \[:1\] (25/08/20)
- Reverting random LMCUT preprocessing to arbitrary \[:2\] (25/08/20)
- Testing combination of multiple tie breaking in LMCUT \[:3\] (25/08/21)
- Randomization as second level tie-breaking for VDM in LMCUT \[:4\] (25/08/21)
- Testing another combination of tie-breaking in LMCUT \[:6\] (25/08/22)
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.5] - 2025/08/14
This is the Master Thesis version (results of this version shown in my Master's Thesis) 
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.4] - 2025/07/19
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Added parameters for cutloop (25/07/17)
### :heavy_plus_sign: Added
- Added test script for testing on small sets of instances (25/07/17)
- Added test scripts for results analysis
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.3] - 2025/07/15
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Missing cutloop time in batch test results script
### :heavy_exclamation_mark: Changed
- Improved Readme
- Improved help output
- Changed cutloop termination policies priorities
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.2] - 2025/07/15
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- First implementation of custom cutloop -> only landmarks and basic termination condition (25/07/09)
- Added questions for Salvagnin and TODOs (25/07/09)
- Added S.E.C. detection for custom cutloop (25/07/14)
- Added In-Out strategy (25/07/14)
- Parametrized Cutloop and In-Out parameters
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.1] - 2025/07/08
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Bug with the closing of flmdet model (25/06/28)
- Removed some magic numbers for better code clarity
### :heavy_exclamation_mark: Changed
- Now compiling with a specific version of CPLEX just needs the root directory of CPLEX
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.2.0] - 2025/06/08
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Fixed bug where if memory limit is reached inside CPLEX, an error is thrown
- Fixed bug where if memory limit is reached and no error is thrown (^), the lower bound is not properly updated
- Fixed bug where in some cases, even with optimal solutions the lowerbound and the incumbent, didn't match
- Fixed bug with memory limit in jobs for cluster
- Fixed bug with showing statistics
- Fixed bug with userhandle in callbacks
### :heavy_exclamation_mark: Changed
- Complete code refactoring
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.1.2] - 2025/06/01
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- MIP model inside relaxation callback is now set on single thread
- Bug where an infinite loop might occur in the relaxation callback
### :heavy_exclamation_mark: Changed
- Using justification graph to separate SEC in candidate callback
- Separating SEC in fractionary solution
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.1.1] - 2025/05/29
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Using random dominant S.E.C. if multiple (equivalent) cycles are found
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.1.0] - 2025/05/28
### Tested all features added in 2.0.*
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Minor visual bugs
- Bug in CMake file where if a CPLEX path is specified, it is ignored in favour of the defaul ones
- Bug where number of landmarks and SEC where not shown if the relaxation callback wasn't used
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.6] - 2025/05/27
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Bug where if no integer solution is found, the lower bound isn't retrieved
### :heavy_exclamation_mark: Changed
- Logger is now thread safe
- Created a copy of the callback data for each thread CPLEX might use
- More meaningful callback statistics (time and cuts added)
### :heavy_plus_sign: Added
- Flag to decide number of threads to use
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.5] - 2025/05/26
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Trying different CPLEX parameters
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.4] - 2025/05/15
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Added thread safety in relaxation callback
- Stronger landmark filtering in relaxation callback
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.3] - 2025/05/11
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Trying violated landmark detection through MIP formulation
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.2] - 2025/05/08
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Improved violated landmark detection of fractionary solution
- Added SEC cuts on fractionary solution
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.1] - 2025/04/25
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Trying ways of producing cuts off the fractionary solution
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [2.0.0] -2025/04/22
### Tested all features added in 1.4.*
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Preparing for ufficial testing and open-sourcing
<!-- ### :heavy_plus_sign: Added -->
### :x: Removed
- Removed Inverse Actions Constraints
<!-- ### :curly_loop: Other -->


## [1.4.2] - 2025/04/18
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Changed algorithm to find cycles
- Removed probably unnecessary constraint in dynamic model
### :heavy_plus_sign: Added
- Handling CPXMIP_MEM_LIM_FEAS status code
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.4.1] - 2025/04/15
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Added inverse actions constraints in dynamic model
- Added sec cuts in dynamic model
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.4.0] - 2025/04/14
### Tested all features added in 1.3.*
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.3.4] - 2025/04/13
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Fixed CMakeLists with Release build type
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Added complete landmark cuts to dynamic model
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.3.3] - 2025/04/03
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Dynamic model might find wrong optimal solutions
<!-- ### :heavy_exclamation_mark: Changed -->
<!-- ### :heavy_plus_sign: Added -->
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.3.2] - 2025/04/02
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Slightly better callback in dynamic model
### :heavy_plus_sign: Added
- Added info executable to see info of an instance
<!-- ### :x: Removed -->
<!-- ### :curly_loop: Other -->


## [1.3.1] - 2025/04/01
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- Imai's model finds wrong optimal solutions
<!-- ### :heavy_exclamation_mark: Changed -->
### :heavy_plus_sign: Added
- Added dynamic model
- Added references
<!-- ### :x: Removed -->
### :curly_loop: Other
- Minor changes


## [1.3.0] - 2025/03/28
### Tested all features added in 1.2.*
<!-- ### :warning: Known issues -->
<!-- ### :ballot_box_with_check: Fixed -->
### :heavy_exclamation_mark: Changed
- Better results reading script
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
### :heavy_plus_sign: Added
- Added status, costs, basic info and other cplex execution info to statistics
    - [~/code/exact/*](code/exact/)
    - [~/code/algorithms.cpp](code/algorithms.cpp)
    - [~/code/hplus_instance.hpp](code/hplus_instance.hpp)
    - [~/code/hplus_instance.cpp](code/hplus_instance.cpp)
    - [~/code/main.cpp](code/main.cpp)
### :x: Removed
- Removed well tested integrity checks
    - [~/code/exact/*](code/exact)
    - [~/code/heuristics/*](code/heuristics)
<!-- ### :curly_loop: Other -->


## [1.2.6] - 2025/03/27
<!-- ### :warning: Known issues -->
### :ballot_box_with_check: Fixed
- "Optimal" solution provided by the model with tb might not be optimal if problem optimization is being used
### :heavy_exclamation_mark: Changed
- Small changes to the CLI parser
    - [~/code/main.cpp](code/main.cpp)
- Smaller number of variables in cplex models
    - [~/code/exact/*](code/exact)
    - [~/code/hplus_instance.hpp](code/hplus_instance.hpp)
    - [~/code/hplus_instance.cpp](code/hplus_instance.cpp)
### :heavy_plus_sign: Added
- Added back tighter bounds in Rankooh's model
    - [~/code/algorithms.cpp](code/algorithms.cpp)
### :x: Removed
- Temporarly removed Rankooh's dynamic model
### :curly_loop: Other
- Changed code organization
- Better [CMakeLists.txt](code/CMakeLists.txt) file


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
- Storing cplex incumbents update in run results
    - [~/code/test/code/results_jobs.py](code/test/code/results_jobs.py)
- Added option to write the lp file
    - [~/code/src/algorithms.cpp](code/src/algorithms.cpp)
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
