#include <math.h>

#include <algorithm>

#include "bs_utils.hpp"
#include "exact.hpp"
#include "fract_separators.hpp"
#include "hplus_algs.hpp"
#include "limits.hxx"
#include "relax_callback.hpp"

inline void init_cutloop(CPXENVptr& env, CPXLPptr& lp) { CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_LP)); }

inline void exit_cutloop(CPXENVptr& env, CPXLPptr& lp) {
    // Set back the problem to being a MIP
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_MILP));
    int ncols{CPXgetnumcols(env, lp)};
    std::vector<int> ind(ncols);
    std::vector<char> types(ncols, 'B');
    std::iota(ind.begin(), ind.end(), 0);
    CPX_HANDLE_CALL(CPXchgctype(env, lp, ncols, ind.data(), types.data()));
}

inline void solve_relaxation(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec, hplus::statistics& stats) {
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else {
        throw timelimit_exception("Reached time limit.");
    }

    CPX_HANDLE_CALL(CPXlpopt(env, lp));

    // Get lowerbound
    switch (const int status{CPXgetstat(env, lp)}) {
        case CPX_STAT_OPTIMAL:
            double cl_lb;
            CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cl_lb));
            stats.lower_bound = std::max(stats.lower_bound, cl_lb);
            break;
        case CPX_STAT_ABORT_TIME_LIM:
            [[fallthrough]];
        case CPX_STAT_ABORT_USER:
            break;
        default:
            LOG_ERROR << "Error in solve_relaxation: unhandled cplex status (" << status << ")";
    }
}

inline auto generate_cuts(CPXENVptr& env, CPXLPptr& lp, const std::vector<double>& relax_point, const hplus::execution& exec,
                          const hplus::instance& inst, hplus::statistics& stats, const std::vector<double>& incumbent, double& inout_w)
    -> unsigned int {
    unsigned int new_cuts{0};

    const auto& add_cuts = [&](std::vector<double> relax_point) {
        // Get info on the relaxation point
        const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

        int nnz{0};
        std::vector<int> ind;
        std::vector<int> begin;
        std::vector<double> val;
        std::vector<double> rhs;
        std::vector<char> sense;

        // Adding landmark as new constraint
        if (exec.fract_cuts.find('m') != std::string::npos) {
            const auto& [found_lm,
                         landmark]{fract_lm_sep::get_r3_violated_landmark(exec, inst, relax_point, stats.total_act_in_lm, stats.total_n_lm)};
            if (found_lm) {
                ind = std::vector<int>(landmark.begin(), landmark.end());
                val = std::vector<double>(landmark.size(), 1.0);
                rhs = std::vector<double>(1, 1);
                sense = std::vector<char>(1, 'G');
                begin = std::vector<int>(1, 0);
                CPX_HANDLE_CALL(
                    CPXaddrows(env, lp, 0, 1, landmark.size(), rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
                stats.cuts_lm++;
                new_cuts++;
            }
        }
        if (exec.fract_cuts.find('l') != std::string::npos) {
            const auto& [found_lm,
                         landmarks]{fract_lm_sep::get_lmcut_violated_landmarks(exec, inst, relax_point, stats.total_act_in_lm, stats.total_n_lm)};
            if (found_lm) {
                for (const auto& landmark : landmarks) {
                    ind = std::vector<int>(landmark.begin(), landmark.end());
                    val = std::vector<double>(landmark.size(), 1.0);
                    rhs = std::vector<double>(1, 1);
                    sense = std::vector<char>(1, 'G');
                    begin = std::vector<int>(1, 0);
                    CPX_HANDLE_CALL(
                        CPXaddrows(env, lp, 0, 1, landmark.size(), rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
                    stats.cuts_lm++;
                    new_cuts++;
                }
            }
        }

        // Adding SEC as new constraint
        if (exec.fract_cuts.find('s') != std::string::npos) {
            const auto& [found_sec, cycles]{relax_cuts::get_violated_sec(inst, fadd_weights)};
            if (found_sec) {
                ind.clear();
                begin.clear();
                val.clear();
                rhs.clear();
                sense = std::vector<char>(cycles.size(), 'L');
                for (const auto& cycle : cycles) {
                    begin.push_back(static_cast<int>(ind.size()));
                    rhs.push_back(static_cast<double>(cycle.size() - 1));
                    std::ranges::copy(cycle,
                                      std::back_inserter(ind));  // labels in the cycle are the indexes for the first adders in the cplex model
                    val.insert(val.end(), cycle.size(), 1.0);
                    nnz += static_cast<int>(cycle.size());
                }
                CPX_HANDLE_CALL(
                    CPXaddrows(env, lp, 0, cycles.size(), nnz, rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
                stats.cuts_sec += cycles.size();
                new_cuts += cycles.size();
            }
        }
    };

    unsigned int inout_it{0};
    double w = inout_w;

    if (exec.inout) {  // In-Out
        while (inout_it <= exec.io_max_iter && new_cuts == 0) {
            inout_it++;
            std::vector<double> inout_relax_point;
            if (inout_it == exec.io_max_iter) {
                w = 0;
            }
            for (unsigned int i = 0; i < relax_point.size(); i++) {
                inout_relax_point.push_back((relax_point[i] * (1 - w)) + (incumbent[i] * w));
            }
            add_cuts(inout_relax_point);
            if (new_cuts == 0) {
                w *= exec.io_weight_update;
            }
        }

        // Dynamic In-Out weight adjustment
        if (inout_it == 1 && inout_w < 0.85) {
            inout_w += 0.1;
        } else if (inout_it > 1) {
            inout_w = w;
        }

    } else {  // Normal
        add_cuts(relax_point);
    }

    return new_cuts;
}

inline void pruning(CPXENVptr& env, CPXLPptr& lp, int base_constraints) {
    const int nrows{CPXgetnumrows(env, lp)};
    if (nrows != base_constraints) {
        std::vector<double> slack(nrows - base_constraints);  // We don't remove base constraints... only those added in the cutloop
        std::vector<int> deleted(nrows, 0);
        CPX_HANDLE_CALL(CPXgetslack(env, lp, slack.data(), base_constraints, nrows - 1));
        for (unsigned int i = 0; i < slack.size(); ++i) {
            if (abs(slack[i]) > HPLUS_EPSILON) {
                deleted[base_constraints + i] = 1;  // If the slack value for row i is not 0, mark that row as to be deleted
            }
        }
        CPX_HANDLE_CALL(CPXdelsetrows(env, lp, deleted.data()));
    }
}

void cutloop::cutloop(CPXENVptr& env, CPXLPptr& lp, hplus::execution& exec, const hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO << "Running custom Cut-Loop";

    double start_time{GET_TIME()};
    stats.cutloop = static_cast<double>(exec.timelimit) - start_time;
    exec.exec_s = hplus::exec_status::CUTLOOP;

    const int base_constraints{CPXgetnumrows(env, lp)};
    unsigned int new_cuts{1};
    unsigned int iteration{0};

    std::vector<double> lb_history;

    const auto& repeat_cutloop = [&]() {
        double current_lb = NAN;
        CPX_HANDLE_CALL(CPXgetobjval(env, lp, &current_lb));
        lb_history.push_back(current_lb);

        // No more cuts
        if (new_cuts == 0) {
            return false;
        }

        // Check gap with incumbent
        if (1 - current_lb / static_cast<double>(inst.sol.cost == 0 ? 1 : inst.sol.cost) <= exec.cl_gap_stop + HPLUS_EPSILON) {
            return false;
        }

        // Do the minimum number of iterations
        if (iteration < exec.cl_min_iter || lb_history.size() <= exec.cl_past_iter) {
            return true;
        }

        // Absolute gap with past iteration (only if old_lb is 0 -> we cannot compute the relative gap if it is 0)
        double old_lb = lb_history[lb_history.size() - exec.cl_past_iter - 1];
        if (old_lb <= HPLUS_EPSILON) {
            return current_lb - old_lb >= HPLUS_EPSILON;
        }

        // Relative gap with past iteration
        double improvement = (current_lb / old_lb) - 1;
        if (improvement < HPLUS_EPSILON) {
            improvement = 0;  // Set to 0 for precision issues
        }
        return improvement >= exec.cl_improv;
    };

    init_cutloop(env, lp);

    binary_set state{inst.n};
    const auto& warm_start{inst.sol.sequence};

    const unsigned int ncols{static_cast<unsigned int>(CPXgetnumcols(env, lp))};
    std::vector<double> incumbent(ncols, 0.0);

    for (const auto& act_i : warm_start) {
        incumbent[act_i] = 1;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) {
                continue;
            }

            unsigned int fadd_idx = inst.m + inst.fadd_cpx_start[act_i] + var_count;
            incumbent[fadd_idx] = 1;
            unsigned int var_idx = inst.m + inst.nfadd + var_i;
            incumbent[var_idx] = 1;
        }
        state |= inst.actions[act_i].eff_sparse;
    }

    double inout_w = exec.io_weight;

    solve_relaxation(env, lp, exec, stats);
    if (VERBOSE_BASIC()) {
        LOG_INFO << "Lower bound at start of cutloop: " << stats.lower_bound;
    }
    while (repeat_cutloop() && !CHECK_STOP()) {
        std::vector<double> relax_point(ncols);
        CPXgetx(env, lp, relax_point.data(), 0, static_cast<int>(ncols - 1));

        // Fix numerical errors
        for (auto& val : relax_point) {
            if (val <= HPLUS_EPSILON) {
                val = 0;
            } else if (val >= 1 - HPLUS_EPSILON) {
                val = 1;
            }
        }

        // Pruning of slack constraints every 5 iterations
        if ((iteration + 1) % 5 == 0 && exec.cl_pruning) {
            pruning(env, lp, base_constraints);
        }

        // Generate new cuts
        double cuts_time = GET_TIME();
        new_cuts = generate_cuts(env, lp, relax_point, exec, inst, stats, incumbent, inout_w);
        stats.relax_callback += GET_TIME() - cuts_time;
        stats.relax_calls++;

        solve_relaxation(env, lp, exec, stats);
        iteration++;
        stats.cutloop_it = iteration;
    }

    if (VERBOSE_BASIC()) {
        LOG_INFO << "Lower bound at end of cutloop: " << stats.lower_bound;
    }

    // Purging of slack constraints (if we exited due to time limit, we might not have a full solution, so pruning constraints might remove more than
    // necessary)
    if (!CHECK_STOP() && exec.cl_pruning) {
        pruning(env, lp, base_constraints);
    }

    stats.const_acyc += CPXgetnumrows(env, lp) - base_constraints;
    stats.cutloop_it = iteration;

    exit_cutloop(env, lp);
    stats.cutloop = GET_TIME() - start_time;

    if (CHECK_STOP()) {
        throw timelimit_exception("");
    }
}
