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
    // TODO (ask) : Ho notato che quando poi farò andare CPXmipopt, rifarà il cutloop... è giusto così o vogliamo evitare che lui rifaccia il cut-loop
    // al nodo radice (visto che è quello che stiamo facendo qua in poche parole...) (thoughtful-sat14-strips-p13_7_86-typed.sas)
    relax_cuts::close_flmdet_model(flmdetenv, flmdetlp);
}

inline void solve_relaxation(CPXENVptr& env, CPXLPptr& lp) { CPX_HANDLE_CALL(CPXlpopt(env, lp)); }

inline bool generate_cuts(CPXENVptr& env, CPXLPptr& lp, CPXENVptr& flmdetenv, CPXLPptr& flmdetlp, const hplus::execution& exec,
                          const hplus::instance& inst) {
    const int ncols{CPXgetnumcols(env, lp)};
    std::vector<double> relax_point(ncols);
    CPXgetx(env, lp, relax_point.data(), 0, ncols - 1);

    // TODO : In-Out strategies
    // LOG_TODO_WARN << "in-out strategy";

    // Get info on the relaxation point
    const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

    // double cost{-1};
    // CPX_HANDLE_CALL(CPXgetobjval(env, lp, &cost));
    // LOG_DEBUG << cost;

    unsigned int new_cuts{0};

    // Adding landmark as new constraint
    const auto& [found, landmark]{relax_cuts::get_violated_landmark(flmdetenv, flmdetlp, exec, inst, relax_point)};
    if (found) {
        std::vector<int> ind(landmark.size());
        int nnz{0};
        for (const auto& act_i : landmark) ind[nnz++] = act_i;
        std::vector<double> val(landmark.size(), 1.0);
        constexpr double rhs{1};
        constexpr char sense{'G'};
        constexpr int begin{0};
        CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs, &sense, &begin, ind.data(), val.data(), nullptr, nullptr));
        new_cuts++;
    }

    // TODO : Add SEC for fractionary solutions

    return new_cuts;
}

void cutloop::cutloop(CPXENVptr& env, CPXLPptr& lp, const hplus::execution& exec, const hplus::instance& inst, hplus::statistics& stats) {
    LOG_INFO << "Running custom Cut-Loop";
    unsigned int iteration{0};
    unsigned int new_cuts{1};

    // Logic for cut-loop termination
    const auto& repeat_cutloop = [&]() { return new_cuts > 0; };

    CPXENVptr flmdetenv = nullptr;
    CPXLPptr flmdetlp = nullptr;
    init_cutloop(exec, env, lp, flmdetenv, flmdetlp, inst);

    while (repeat_cutloop() && !CHECK_STOP()) {
        solve_relaxation(env, lp);
        new_cuts = generate_cuts(env, lp, flmdetenv, flmdetlp, exec, inst);
        stats.const_acyc += new_cuts;
        iteration++;
    }

    exit_cutloop(env, lp, flmdetenv, flmdetlp);
}