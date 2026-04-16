#include <format>
#include <logger.hxx>

#include "exact.hpp"
#include "utils.hpp"

void ts::build_model(hplus::execution& exec, hplus::instance& inst, hplus::statistics& stats, CPXENVptr& env, CPXLPptr& lp) {
    if (VERBOSE_BASIC()) LOG_INFO << "Building timestep (TS) model";

    const auto stopcheck = [&]() {
        if (CHECK_STOP()) [[unlikely]]
            throw timelimit_exception("Reached time limit.");
    };

    const unsigned int T = std::min(inst.n, inst.m);

    // ====================================================== //
    // =================== CPLEX VARIABLES ================== //
    // ====================================================== //

    // Variable layout:
    //   y_a      : col a                                                  (m variables)
    //   x_{apt}  : col m + fadd_cpx_start[a]*T + eff_k*T + t             (nfadd*T variables)
    //   x_{pt}   : col m + nfadd*T + p*T + t                             (n*T variables)
    std::vector<double> objs(inst.m);
    std::vector<double> lbs(inst.m, 0.0);
    std::vector<double> ubs(inst.m, 1.0);
    std::vector<char> types(inst.m, 'B');
    std::vector<std::string> names(inst.m);
    std::vector<char*> c_names(inst.m);

    // --- y_a: action-used variables (objective = cost_a) --- //
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        objs[act_i] = static_cast<double>(inst.actions[act_i].cost);
        names[act_i] = std::format("act{}", act_i);
        c_names[act_i] = names[act_i].data();
    }
    CPX_HANDLE_CALL(CPXnewcols(env, lp, inst.m, objs.data(), lbs.data(), ubs.data(), types.data(), c_names.data()));
    stats.var_base += inst.m;
    stopcheck();

    objs.clear();
    objs.resize(T, 0.0);
    lbs.clear();
    lbs.resize(T, 0.0);
    ubs.clear();
    ubs.resize(T, 1.0);
    types.clear();
    types.resize(T, 'B');
    names.clear();
    names.resize(T);
    c_names.clear();
    c_names.resize(T);

    // --- x_{apt}: action-effect-time variables (unnamed, added in batches) --- //
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        for (unsigned int idx = 0; idx < inst.actions[act_i].eff_sparse.size(); ++idx) {
            for (unsigned int time = 0; time < T; ++time) {
                names[time] = std::format("fadd{}x{}x{}", act_i, inst.actions[act_i].eff_sparse.at(idx), time);
                c_names[time] = names[time].data();
            }
            CPX_HANDLE_CALL(CPXnewcols(env, lp, T, objs.data(), lbs.data(), ubs.data(), types.data(), c_names.data()));
            stopcheck();
            stats.var_acyc += T;
        }
    }

    // --- x_{pt}: fact-time variables (unnamed, added in batches) --- //
    for (unsigned int fact_i = 0; fact_i < inst.n; ++fact_i) {
        for (unsigned int time = 0; time < T; ++time) {
            names[time] = std::format("var{}x{}", fact_i, time);
            c_names[time] = names[time].data();
        }
        CPX_HANDLE_CALL(CPXnewcols(env, lp, T, objs.data(), lbs.data(), ubs.data(), types.data(), c_names.data()));
        stopcheck();
        stats.var_acyc += T;
    }

    // ====================================================== //
    // ================== CPLEX CONSTRAINTS ================= //
    // ====================================================== //

    const unsigned int fadd_start = inst.m;
    const unsigned int var_start = inst.m + (inst.nfadd * T);

    const auto get_act_idx = [](unsigned int a) -> int { return static_cast<int>(a); };
    const auto get_fadd_idx = [&](unsigned int a, unsigned int eff_k, unsigned int t) -> int {
        return static_cast<int>(fadd_start + (inst.fadd_cpx_start[a] * T) + (eff_k * T) + t);
    };
    const auto get_var_idx = [&](unsigned int p, unsigned int t) -> int { return static_cast<int>(var_start + (p * T) + t); };

    constexpr char sense_e{'E'};
    constexpr char sense_l{'L'};
    constexpr char sense_g{'G'};
    constexpr double rhs_0{0.0};
    constexpr double rhs_1{1.0};
    constexpr int begin{0};

    std::vector<int> ind(1);
    std::vector<double> val(1, 1.0);

    // --- Fixed actions: y_a >= 1 --- //
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        if (!inst.fixed_actions[act_i]) {
            continue;
        }
        ind[0] = get_act_idx(act_i);
        CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 1, &rhs_1, &sense_g, &begin, ind.data(), val.data(), nullptr, nullptr));
        stats.const_base++;
    }
    stopcheck();

    ind.clear();
    ind.resize(T);
    val.clear();
    val.resize(T, 1.0);

    // --- Goal / fixed facts: sum_t x_{pt} = 1; non-goal: sum_t x_{pt} <= 1 --- //
    for (unsigned int fact_i = 0; fact_i < inst.n; ++fact_i) {
        for (unsigned int t = 0; t < T; ++t) {
            ind[t] = get_var_idx(fact_i, t);
        }
        const char sense = (inst.goal[fact_i] || inst.fixed_facts[fact_i]) ? sense_e : sense_l;
        CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, T, &rhs_1, &sense, &begin, ind.data(), val.data(), nullptr, nullptr));
        stats.const_base++;
        stopcheck();
    }

    ind.clear();
    ind.resize(inst.n + 1);
    val.clear();
    val.resize(inst.n + 1, 1.0);

    // --- Consistency: sum_{a: p in eff(a)} x_{apt} - x_{pt} = 0 for each (p, t) ---
    // Precompute eff_k (position of p in eff_sparse[a]) for each (a, p) pair to avoid
    // repeated std::find inside the inner loop.
    std::vector<std::vector<unsigned int>> eff_k_for_p(inst.n);
    for (unsigned int p = 0; p < inst.n; ++p) {
        const auto& acts = inst.act_with_eff[p];
        eff_k_for_p[p].reserve(acts.size());
        for (const auto& a : acts) {
            unsigned int eff_k = static_cast<unsigned int>(std::find(inst.actions[a].eff_sparse.begin(), inst.actions[a].eff_sparse.end(), p) -
                                                           inst.actions[a].eff_sparse.begin());
            eff_k_for_p[p].push_back(eff_k);
        }
    }

    for (unsigned int p = 0; p < inst.n; ++p) {
        const auto& acts = inst.act_with_eff[p];
        for (unsigned int t = 0; t < T; ++t) {
            unsigned int nnz = 0;
            for (unsigned int k = 0; k < acts.size(); ++k) {
                ind[nnz] = get_fadd_idx(acts[k], eff_k_for_p[p][k], t);
                val[nnz] = 1.0;
                ++nnz;
            }
            ind[nnz] = get_var_idx(p, t);
            val[nnz] = -1.0;
            ++nnz;
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_e, &begin, ind.data(), val.data(), nullptr, nullptr));
            stats.const_base++;
        }
        stopcheck();
    }

    ind.clear();
    ind.resize(2);
    val = {1.0, -1.0};

    // --- Linking: x_{apt} - y_a <= 0 for each (a, eff_k, t) --- //
    for (unsigned int a = 0; a < inst.m; ++a) {
        for (unsigned int eff_k = 0; eff_k < inst.actions[a].eff_sparse.size(); ++eff_k) {
            for (unsigned int t = 0; t < T; ++t) {
                ind[0] = get_fadd_idx(a, eff_k, t);
                ind[1] = get_act_idx(a);
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 2, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
                stats.const_base++;
            }
        }
        stopcheck();
    }

    ind.clear();
    ind.resize(T + inst.m);
    val.clear();
    val.resize(T + inst.m, 1.0);

    // --- Ordering
    for (unsigned int p = 0; p < inst.n; ++p) {
        for (unsigned int q = 0; q < inst.n; ++q) {
            for (unsigned int t = 0; t < T; ++t) {
                int nnz = 0;
                const auto& acts = inst.act_with_eff[q];
                for (unsigned int k = 0; k < acts.size(); ++k) {
                    if (!inst.actions[acts[k]].pre[p]) {
                        continue;
                    }
                    ind[nnz] = get_fadd_idx(acts[k], eff_k_for_p[q][k], t);
                    val[nnz] = 1.0;
                    ++nnz;
                }
                for (unsigned int tprime = 0; tprime < t; ++tprime) {
                    ind[nnz] = get_var_idx(p, tprime);
                    val[nnz] = -1.0;
                    ++nnz;
                }
                if (nnz == 0) {
                    continue;
                }
                ASSERT(nnz <= T + inst.m);
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, nnz, &rhs_0, &sense_l, &begin, ind.data(), val.data(), nullptr, nullptr));
                stopcheck();
                stats.const_acyc++;
            }
        }
    }

    // --- Landmarks (only when preprocessing is active): sum_{a in L} y_a >= 1 --- //
    if (exec.prep) {
        for (const auto& landmark : inst.landmarks) {
            if (landmark.size() == 1) {
                std::vector<int> ind = {get_act_idx(landmark[0])};
                std::vector<double> val = {1.0};
                CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, 1, &rhs_1, &sense_g, &begin, ind.data(), val.data(), nullptr, nullptr));
                stats.const_base++;
                continue;
            }
            std::vector<int> ind(landmark.size());
            std::vector<double> val(landmark.size(), 1.0);
            for (unsigned int k = 0; k < landmark.size(); ++k) ind[k] = get_act_idx(landmark[k]);
            CPX_HANDLE_CALL(CPXaddrows(env, lp, 0, 1, landmark.size(), &rhs_1, &sense_g, &begin, ind.data(), val.data(), nullptr, nullptr));
            stats.const_base++;
            stopcheck();
        }
    }
}

void ts::post_warm_start(const hplus::execution& exec, hplus::instance& inst, CPXENVptr& env, CPXLPptr& lp) {
    if (VERBOSE_BASIC()) LOG_INFO << "Posting warm start to TS model";

    const unsigned int T = std::min(inst.n, inst.m);
    const auto& warm_start{inst.sol.sequence};

    constexpr int izero{0};
    constexpr int effortlevel{CPX_MIPSTART_CHECKFEAS};

    // Variable layout mirrors build_model:
    //   y_a      : col a
    //   x_{apt}  : col m + fadd_cpx_start[a]*T + eff_k*T + t
    //   x_{pt}   : col m + nfadd*T + p*T + t

    const unsigned int fadd_start = inst.m;
    const unsigned int var_start = inst.m + (inst.nfadd * T);

    const auto get_act_idx = [](unsigned int a) -> unsigned int { return a; };
    const auto get_fadd_idx = [&](unsigned int a, unsigned int eff_k, unsigned int t) -> unsigned int {
        return fadd_start + (inst.fadd_cpx_start[a] * T) + (eff_k * T) + t;
    };
    const auto get_var_idx = [&](unsigned int p, unsigned int t) -> unsigned int { return var_start + (p * T) + t; };

    // Replay the warm start: record the timestep at which each fact is first achieved.
    // first_time[p] = t means x_{pt} = 1 and x_{apt} = 1 for the action that first-achieved p.
    // Facts never achieved keep first_time = UINT_MAX.
    binary_set state{inst.n};
    std::vector<unsigned int> first_time(inst.n, UINT_MAX);
    // first_achiever[p] = {action_idx, eff_k} for the action that first-achieved p
    std::vector<std::pair<unsigned int, unsigned int>> first_achiever(inst.n, {UINT_MAX, UINT_MAX});
    unsigned int step{0};

    // Build sparse MIP start: only non-zero entries.
    std::vector<int> ind;
    std::vector<double> val;
    ind.reserve(warm_start.size() + (inst.n * 2));
    val.reserve(warm_start.size() + (inst.n * 2));

    for (const auto act_i : warm_start) {
        ind.push_back(static_cast<int>(get_act_idx(act_i)));
        val.push_back(1.0);
        unsigned int eff_k = 0;
        for (const auto& p : inst.actions[act_i].eff_sparse) {
            if (!state[p]) {
                first_time[p] = step;
                first_achiever[p] = {act_i, eff_k};
            }
            ++eff_k;
        }
        state |= inst.actions[act_i].eff;
        ++step;
    }

    // Set x_{pt} = 1 and x_{apt} = 1 for each achieved fact p at its first_time.
    for (unsigned int p = 0; p < inst.n; ++p) {
        const unsigned int t = first_time[p];
        if (t == UINT_MAX) {
            continue;
        }
        ind.push_back(static_cast<int>(get_var_idx(p, t)));
        val.push_back(1.0);
        const auto& [act_i, eff_k] = first_achiever[p];
        ind.push_back(static_cast<int>(get_fadd_idx(act_i, eff_k, t)));
        val.push_back(1.0);
    }

    const int nnz = static_cast<int>(ind.size());
    CPX_HANDLE_CALL(CPXaddmipstarts(env, lp, 1, nnz, &izero, ind.data(), val.data(), &effortlevel, nullptr));
}
