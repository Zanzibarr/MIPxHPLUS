#include "relax_callback.hpp"

#include <set>

[[nodiscard]]
std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> relax_cuts::relaxationpoint_info(const hplus::instance& inst,
                                                                                                              std::vector<double>& relax_point) {
    std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> fadd_weights;
    // Handle precision errors (i.e.: -1e^-06 is to be considered 0)
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
            // Store the weights of the first achievers in a more convinient way
            fadd_weights[{act_i, inst.actions[act_i].eff_sparse[i]}] = relax_point[idx];
        }
    }

    return fadd_weights;
}

void callbacks::relaxation_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst, thread_data& data) {
    int nodeuid{-1}, nodedepth{-1};
    CPX_HANDLE_CALL(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEUID, &nodeuid));
    CPX_HANDLE_CALL(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEDEPTH, &nodedepth));
    static std::set<int> visited_nodes;
    // If we have our custom cutloop in place, we don't need to generate cuts from fractionary solutions in the first root node relaxation
    if (exec.custom_cutloop && nodeuid == 0) return;
    // If we don't want cut at nodes we exit
    if (!exec.fract_cuts_at_nodes && nodedepth != 0) return;
    // If we already called this callback from this node, we skip (unless we are in a root node... in this case we try CPLEX's cutloop)
    if (nodedepth != 0 && visited_nodes.count(nodeuid) > 0) return;
    visited_nodes.insert(nodeuid);

    data.relax_calls++;

    double start_time = GET_TIME();

    std::vector<double> relax_point(inst.m + inst.nfadd);
    double _{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXcallbackgetrelaxationpoint(context, relax_point.data(), 0, inst.m + inst.nfadd - 1, &_));

    const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

    // exec.fract_cuts is a string containing one letter per type of cut to be applied
    // -> l : landmark cuts
    // -> s : SEC
    try {
        if (exec.fract_cuts.find('l') != std::string::npos)
            data.usercuts_lm += relax_cuts::lm(context, data.flmdet_env, data.flmdet_lp, exec, inst, relax_point, data);
        if (exec.fract_cuts.find('s') != std::string::npos) data.usercuts_sec += relax_cuts::sec(context, inst, fadd_weights);
    } catch (timelimit_exception& e) {
        return;
    }

    // Update the candidate time
    data.relax_time += GET_TIME() - start_time;
}
