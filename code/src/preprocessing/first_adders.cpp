#include <binary_set.hxx>

#include "solver.hpp"

auto Solver::prep_first_adders_(std::vector<std::vector<unsigned int>>& landmarks) -> bool {
    logger_[DEBUG] << "PREP: first adders extraction";

    BinarySet fact_lm_for_a(inst_.n);
    unsigned int erased{0};

    for (unsigned int act_i = 0; act_i < inst_.m; act_i++) {
        fact_lm_for_a.clear();
        for (const auto& pre : inst_.actions[act_i].pre_sparse) {
            for (const auto& fact_q : landmarks[pre]) {
                fact_lm_for_a.add(fact_q);
            }
        }
        erased += static_cast<unsigned int>(
            std::erase_if(inst_.actions[act_i].eff_sparse, [&fact_lm_for_a](const auto val) { return fact_lm_for_a[val]; }));

        if (global_limits::time_reached()) [[unlikely]] {
            throw EarlyExit("preprocessing first adders extraction", EarlyExit::TIMELIMIT);
        }
    }
    inst_.nfadd -= erased;
    return erased > 0;
}
