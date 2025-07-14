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

inline bool generate_cuts(CPXENVptr& env, CPXLPptr& lp, CPXENVptr& flmdetenv, CPXLPptr& flmdetlp, const hplus::execution& exec,
                          const hplus::instance& inst) {
    const int ncols{CPXgetnumcols(env, lp)};
    std::vector<double> relax_point(ncols);
    CPXgetx(env, lp, relax_point.data(), 0, ncols - 1);

    // TODO : In-Out strategies
    // LOG_TODO_WARN << "in-out strategy";

    // Get info on the relaxation point
    const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

    if (exec.verbosity == hplus::verbose::DEBUG) {
        double cost{-1};
        CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cost));
        LOG_DEBUG << cost;
    }

    unsigned int new_cuts{0};
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
        CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, landmark.size(), rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
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
        CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, cycles.size(), nnz, rhs.data(), sense.data(), begin.data(), ind.data(), val.data(), nullptr, nullptr));
        new_cuts += cycles.size();
    }

    return new_cuts;
}

inline void pruning(CPXENVptr& env, CPXLPptr& lp, int base_constraints) {
    // TODO (ask) : Rimuovendo questi constraints nel cutloop di CPLEX perdiamo il lowerbound...
    const int nrows{CPXgetnumrows(env, lp)};
    if (nrows != base_constraints) {
        std::vector<double> slack(nrows - base_constraints);  // We don't remove base constraints... only those added in the cutloop
        std::vector<int> deleted(nrows, 0);
        CPX_HANDLE_CALL(CPXgetslack(env, lp, slack.data(), base_constraints + 1, nrows - 1));
        for (unsigned int i = 0; i < slack.size(); ++i) {
            if (abs(slack[i]) > HPLUS_EPSILON)
                deleted[base_constraints + i] = 1;  // If the slack value for row i is not 0, mark that row as to be deleted
        }
        CPX_HANDLE_CALL(CPXdelsetrows(env, lp, deleted.data()));
    }

    LOG_DEBUG << "Added " << nrows - base_constraints << " rows";
    LOG_DEBUG << "Deleted " << nrows - CPXgetnumrows(env, lp) << " rows";
}

void cutloop::cutloop(CPXENVptr& env, CPXLPptr& lp, hplus::execution& exec, const hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO << "Running custom Cut-Loop";

    double start_time{GET_TIME()};
    stats.cutloop = static_cast<double>(exec.timelimit) - start_time;
    exec.exec_s = hplus::exec_status::CUTLOOP;

    const int base_constraints{CPXgetnumrows(env, lp)};
    unsigned int iteration{0};
    unsigned int new_cuts{1};

    // Logic for cut-loop termination
    const auto& repeat_cutloop = [&]() { return new_cuts > 0; };

    CPXENVptr flmdetenv = nullptr;
    CPXLPptr flmdetlp = nullptr;
    init_cutloop(exec, env, lp, flmdetenv, flmdetlp, inst);

    solve_relaxation(env, lp, exec);
    while (repeat_cutloop() && !CHECK_STOP()) {
        new_cuts = generate_cuts(env, lp, flmdetenv, flmdetlp, exec, inst);
        solve_relaxation(env, lp, exec);
        stats.const_acyc += new_cuts;
        iteration++;
    }

    if (CHECK_STOP()) {
        exit_cutloop(env, lp, flmdetenv, flmdetlp);
        throw timelimit_exception("");
    }

    // Purging of slack constraints
    // pruning(env, lp, base_constraints);

    exit_cutloop(env, lp, flmdetenv, flmdetlp);
    stats.cutloop = GET_TIME() - start_time;
}