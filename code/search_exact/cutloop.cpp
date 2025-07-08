#include "../cut_separators/relax_callback.hpp"
#include "exact.hpp"

inline void init_cutloop(CPXENVptr& env, CPXLPptr& lp) {
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_LP));
    // TODO (ask) : Is this enough?
}

inline void exit_cutloop(CPXENVptr& env, CPXLPptr& lp) {
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_MILP));
    // TODO (ask) : Maybe the fixed variables are not fixed anymore???
}

inline void solve_relaxation(CPXENVptr& env, CPXLPptr& lp) { CPX_HANDLE_CALL(CPXlpopt(env, lp)); }

inline bool generate_cuts(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& inst) {
    const int ncols{CPXgetnumcols(env, lp)};
    std::vector<double> xstar(ncols);
    CPXgetx(env, lp, xstar.data(), 0, ncols - 1);

    // TODO : In-Out strategies
    LOG_TODO_WARN;

    // Get info on the relaxation point
    const auto& fadd_weights = callbacks::relaxationpoint_info(inst, xstar);

    // TODO : Generate cuts
    unsigned int new_cuts{0};
    LOG_TODO;

    return new_cuts;
}

void cutloop::cutloop(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& inst, hplus::statistics& stats) {
    unsigned int iteration{0};
    unsigned int new_cuts{1};

    // Logic for cut-loop termination
    const auto& repeat_cutloop = [&]() { return iteration < 20 && new_cuts > 0; };

    init_cutloop(env, lp);

    while (repeat_cutloop()) {
        solve_relaxation(env, lp);
        new_cuts = generate_cuts(env, lp, inst);
        stats.const_acyc += new_cuts;
        iteration++;
    }

    exit_cutloop(env, lp);
}