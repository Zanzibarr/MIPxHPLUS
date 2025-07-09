#include "../cut_separators/relax_callback.hpp"
#include "exact.hpp"

inline void init_cutloop(CPXENVptr& env, CPXLPptr& lp) { CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_LP)); }

inline void exit_cutloop(CPXENVptr& env, CPXLPptr& lp) {
    // Set back the problem to being a MIP
    CPX_HANDLE_CALL(CPXchgprobtype(env, lp, CPXPROB_MILP));
    int ncols{CPXgetnumcols(env, lp)};
    std::vector<int> ind(ncols);
    std::vector<char> types(ncols, 'B');
    std::iota(ind.begin(), ind.end(), 0);
    CPX_HANDLE_CALL(CPXchgctype(env, lp, ncols, ind.data(), types.data()));
    // TODO (ask) : Al momento non ho ancora aggiunto tagli, ma ho notato che quando poi farò andare CPXmipopt, rifarà il cutloop... è giusto così o
    // vogliamo evitare che lui rifaccia il cut-loop al nodo radice (visto che è quello che stiamo facendo qua in poche parole...)
    // (thoughtful-sat14-strips-p13_7_86-typed.sas)
}

inline void solve_relaxation(CPXENVptr& env, CPXLPptr& lp) { CPX_HANDLE_CALL(CPXlpopt(env, lp)); }

inline bool generate_cuts(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& inst) {
    const int ncols{CPXgetnumcols(env, lp)};
    std::vector<double> relaxed_solution(ncols);
    CPXgetx(env, lp, relaxed_solution.data(), 0, ncols - 1);

    // TODO : In-Out strategies
    LOG_TODO_WARN << "in-out strategy";

    double cost{-1};
    CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cost));
    LOG_DEBUG << cost;

    // Get info on the relaxation point
    const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relaxed_solution);

    // TODO : Generate cuts
    unsigned int new_cuts{0};
    LOG_TODO_WARN << "cuts generation";

    return new_cuts;
}

void cutloop::cutloop(CPXENVptr& env, CPXLPptr& lp, const hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO << "Running custom Cut-Loop";
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