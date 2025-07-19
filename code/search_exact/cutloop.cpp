#include "../cut_separators/relax_callback.hpp"
#include "exact.hpp"

inline void init_cutloop(const hplus::execution& exec, CPXENVptr& env, CPXLPptr& lp, CPXENVptr& flmdetenv, CPXLPptr& flmdetlp,
                         const hplus::instance& inst) {
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_LP));
    relax_cuts::create_flmdet_model(inst, flmdetenv, flmdetlp, exec.threads);
}

inline void exit_cutloop(CPXENVptr& env, CPXLPptr& lp, CPXENVptr& flmdetenv, CPXLPptr& flmdetlp) {
    // Set back the problem to being a MIP
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_MILP));
    int ncols{CPXgetnumcols(env, lp)};
    std::vector<int> ind(ncols);
    std::vector<char> types(ncols, 'B');
    std::iota(ind.begin(), ind.end(), 0);
    CPX_HANDLE_CALL(CPXchgctype(env, lp, ncols, ind.data(), types.data()));
    relax_cuts::close_flmdet_model(flmdetenv, flmdetlp);
}

inline void solve_relaxation(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec) {
    if (exec.timelimit > 0 && static_cast<double>(exec.timelimit) > GET_TIME()) {
        CPX_HANDLE_CALL(CPXsetdblparam(env, CPXPARAM_TimeLimit, static_cast<double>(exec.timelimit) - GET_TIME()));
    } else
        throw timelimit_exception("Reached time limit.");

    CPX_HANDLE_CALL(CPXlpopt(env, lp));
}

inline unsigned int generate_cuts(CPXENVptr& env, CPXLPptr& lp, CPXENVptr& flmdetenv, CPXLPptr& flmdetlp, const hplus::execution& exec,
                                  const hplus::instance& inst, const std::vector<double>& incumbent, double& inout_w) {
    const int ncols{CPXgetnumcols(env, lp)};
    std::vector<double> relax_point(ncols);
    CPXgetx(env, lp, relax_point.data(), 0, ncols - 1);

    unsigned int new_cuts{0};

    const auto& add_cuts = [&](std::vector<double> relax_point) {
        // Get info on the relaxation point
        const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

        int nnz{0};
        std::vector<int> ind, begin;
        std::vector<double> val, rhs;
        std::vector<char> sense;

        // Adding landmark as new constraint
        const auto& [found_lm, landmark]{relax_cuts::get_violated_landmark(flmdetenv, flmdetlp, exec, inst, relax_point)};
        if (found_lm) {
            ind = std::vector<int>(landmark.begin(), landmark.end());
            val = std::vector<double>(landmark.size(), 1.0);
            rhs = std::vector<double>(1, 1);
            sense = std::vector<char>(1, 'G');
            begin = std::vector<int>(1, 0);
            CPX_HANDLE_CALL(
                CPXaddrows(env, lp, 0, 1, landmark.size(), rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
            new_cuts++;
        }

        // Adding SEC as new constraint
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
                std::copy(cycle.begin(), cycle.end(),
                          std::back_inserter(ind));  // labels in the cycle are the indexes for the first adders in the cplex model
                val.insert(val.end(), cycle.size(), 1.0);
                nnz += cycle.size();
            }
            CPX_HANDLE_CALL(
                CPXaddrows(env, lp, 0, cycles.size(), nnz, rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
            new_cuts += cycles.size();
        }
    };

    unsigned int inout_it{0}, max_inout_it{exec.io_max_iter};
    double w = inout_w, w_update = exec.io_weight_update;

    if (exec.inout) {  // In Out
        while (inout_it <= max_inout_it && new_cuts == 0) {
            inout_it++;
            std::vector<double> inout_relax_point;
            if (inout_it == max_inout_it) w = 0;
            for (int i = 0; i < ncols; i++) inout_relax_point.push_back(relax_point[i] * (1 - w) + incumbent[i] * w);
            add_cuts(inout_relax_point);
            if (new_cuts == 0) w *= w_update;
        }
        // if (inout_it == 1 && inout_w < 0.85)
        //     inout_w += 0.1;
        // else if (inout_it > 1)
        //     inout_w = w;
    } else  // Normal
        add_cuts(relax_point);

    LOG_DEBUG << inout_w;

    return new_cuts;
}

inline void pruning(CPXENVptr& env, CPXLPptr& lp, int base_constraints) {
    const int nrows{CPXgetnumrows(env, lp)};
    if (nrows != base_constraints) {
        std::vector<double> slack(nrows - base_constraints);  // We don't remove base constraints... only those added in the cutloop
        std::vector<int> deleted(nrows, 0);
        CPX_HANDLE_CALL(CPXgetslack(env, lp, slack.data(), base_constraints, nrows - 1));
        for (unsigned int i = 0; i < slack.size(); ++i) {
            if (abs(slack[i]) > HPLUS_EPSILON)
                deleted[base_constraints + i] = 1;  // If the slack value for row i is not 0, mark that row as to be deleted
        }
        CPX_HANDLE_CALL(CPXdelsetrows(env, lp, deleted.data()));
    }

    LOG_DEBUG << "Pruning: Added " << nrows - base_constraints << " rows; Deleted " << nrows - CPXgetnumrows(env, lp) << " rows";
}

void cutloop::cutloop(CPXENVptr& env, CPXLPptr& lp, hplus::execution& exec, const hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO << "Running custom Cut-Loop";

    double start_time{GET_TIME()};
    stats.cutloop = static_cast<double>(exec.timelimit) - start_time;
    exec.exec_s = hplus::exec_status::CUTLOOP;

    const int base_constraints{CPXgetnumrows(env, lp)};
    unsigned int new_cuts{1}, iteration{0};

    std::vector<double> lb_history;
    const double k_percent = exec.cl_improv;
    const unsigned int lookback_iterations = exec.cl_past_iter, min_iteration = exec.cl_min_iter;

    const auto& repeat_cutloop = [&]() {
        double current_lb;
        CPX_HANDLE_CALL(CPXgetobjval(env, lp, &current_lb));
        lb_history.push_back(current_lb);

        LOG_DEBUG << "Current lb:      " << current_lb;

        // No more cuts
        if (new_cuts == 0) return false;

        // Check gap with incumbent
        if (1 - current_lb / static_cast<double>(inst.sol.cost == 0 ? 1 : inst.sol.cost) <= exec.cl_gap_stop + HPLUS_EPSILON) return false;

        // Do the minimum number of iterations
        if (iteration < min_iteration || lb_history.size() <= lookback_iterations) return true;

        // Absolute gap with past iteration
        double old_lb = lb_history[lb_history.size() - lookback_iterations - 1];
        if (old_lb < 1e-9) return current_lb - old_lb >= HPLUS_EPSILON;

        // Relative gap with past iteration
        double improvement = (current_lb - old_lb) / old_lb;
        if (improvement < HPLUS_EPSILON) improvement = 0;  // Fix for precision issues
        return improvement >= k_percent;
    };

    CPXENVptr flmdetenv = nullptr;
    CPXLPptr flmdetlp = nullptr;
    init_cutloop(exec, env, lp, flmdetenv, flmdetlp, inst);

    binary_set state{inst.n};
    const auto& warm_start{inst.sol.sequence};

    const unsigned int ncols{static_cast<unsigned int>(CPXgetnumcols(env, lp))};
    std::vector<double> incumbent(ncols, 0.0);

    for (const auto& act_i : warm_start) {
        incumbent[act_i] = 1;
        int var_count{-1};
        for (const auto& var_i : inst.actions[act_i].eff_sparse) {
            var_count++;
            if (state[var_i]) continue;

            unsigned int fadd_idx = inst.m + inst.fadd_cpx_start[act_i] + var_count;
            incumbent[fadd_idx] = 1;
            unsigned int var_idx = inst.m + inst.nfadd + var_i;
            incumbent[var_idx] = 1;
        }
        state |= inst.actions[act_i].eff;
    }

    double inout_w = exec.io_weight;

    solve_relaxation(env, lp, exec);
    while (repeat_cutloop() && !CHECK_STOP()) {
        // Purging of slack constraints every 5 iterations
        if ((iteration + 1) % 5 == 0 && exec.cl_pruning) pruning(env, lp, base_constraints);
        new_cuts = generate_cuts(env, lp, flmdetenv, flmdetlp, exec, inst, incumbent, inout_w);
        solve_relaxation(env, lp, exec);
        iteration++;
    }

    double tmp_lb;
    CPX_HANDLE_CALL(CPXgetobjval(env, lp, &tmp_lb));
    LOG_DEBUG << "Lower bound after cutloop : " << tmp_lb;

    // Purging of slack constraints (if we exited due to time limit, we might not have a full solution, so pruning constraints might remove more than
    // necessary)
    if (!CHECK_STOP() && exec.cl_pruning) pruning(env, lp, base_constraints);

    stats.const_acyc += CPXgetnumrows(env, lp) - base_constraints;
    stats.cutloop_it = iteration;

    exit_cutloop(env, lp, flmdetenv, flmdetlp);
    stats.cutloop = GET_TIME() - start_time;

    if (CHECK_STOP()) throw timelimit_exception("");
}
