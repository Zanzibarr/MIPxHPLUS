#include "relax_callback.hpp"

[[nodiscard]]
std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> relax_cuts::relaxationpoint_info(const hplus::instance& inst,
                                                                                                              std::vector<double>& relax_point) {
    std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> fadd_weights;
    for (unsigned int idx = 0; idx < inst.m; ++idx) {
        if (relax_point[idx] <= HPLUS_EPSILON)
            relax_point[idx] = 0.0;
        else if (relax_point[idx] >= 1 - HPLUS_EPSILON)
            relax_point[idx] = 1.0;
    }
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); ++i) {
            unsigned int idx = inst.m + inst.fadd_cpx_start[act_i] + i;
            if (relax_point[idx] <= HPLUS_EPSILON)
                relax_point[idx] = 0.0;
            else if (relax_point[idx] >= 1 - HPLUS_EPSILON)
                relax_point[idx] = 1.0;
            fadd_weights[{act_i, inst.actions[act_i].eff_sparse[i]}] = relax_point[idx];
        }
    }

    return fadd_weights;
}

void callbacks::relaxation_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, thread_data& data) {
    // Get the current time
    double start_time = GET_TIME();

    std::vector<double> relax_point(inst.m + inst.nfadd);
    double _{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXcallbackgetrelaxationpoint(context, relax_point.data(), 0, inst.m + inst.nfadd - 1, &_));

    // Get info on the relaxation point
    const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

    if (exec.fract_cuts.find('l') != std::string::npos)
        data.usercuts_lm += relax_cuts::lm(context, data.flmdet_env, data.flmdet_lp, exec, inst, relax_point);
    if (exec.fract_cuts.find('s') != std::string::npos) data.usercuts_sec += relax_cuts::sec(context, inst, fadd_weights);

    // Update the candidate time
    data.relax_time += GET_TIME() - start_time;
}
