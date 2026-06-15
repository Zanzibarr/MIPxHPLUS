#include "relax_callback.hpp"

#include <atomic>
#include <limits.hxx>
#include <mutex>
#include <unordered_set>

#include "stats_registry.hxx"
#include "timer.hxx"
#include "utils.hpp"

namespace {
std::atomic<bool> lb_rootnode_recorded{false};
std::atomic<double> last_root_lb{-1.0};
}  // namespace

void callbacks::ensure_lb_rootnode(double best_lb) {
    bool expected = false;
    if (lb_rootnode_recorded.compare_exchange_strong(expected, true)) {
        STATS.gauge_record<"lb_rootnode">(best_lb);
    }
}

[[nodiscard]]
auto relax_cuts::relaxationpoint_info(const hplus::instance& inst, std::vector<double>& relax_point)
    -> std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> {
    std::unordered_map<std::pair<unsigned int, unsigned int>, double, pair_hash> fadd_weights;
    for (unsigned int act_i = 0; act_i < inst.m; ++act_i) {
        for (unsigned int i = 0; i < inst.actions[act_i].eff_sparse.size(); ++i) {
            unsigned int idx = inst.m + inst.fadd_cpx_start[act_i] + i;
            if (relax_point[idx] <= HPLUS_EPSILON) {
                relax_point[idx] = 0.0;
            } else if (relax_point[idx] >= 1 - HPLUS_EPSILON) {
                relax_point[idx] = 1.0;
            }
            // Store the weights of the first achievers in a more convinient way
            fadd_weights[{act_i, inst.actions[act_i].eff_sparse[i]}] = relax_point[idx];
        }
    }

    return fadd_weights;
}

void callbacks::relaxation_callback(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst) {
    // If the cutloop is enabled, the first relaxation has already been stored
    if (!exec.custom_cutloop) {
        static std::once_flag lb_once;
        std::call_once(lb_once, [&] {
            double best_lb{-1};
            CPX_HANDLE_CALL(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));
            STATS.gauge_record<"lb_relaxation">(best_lb);
        });
    }

    int nodeuid{-1};
    int nodedepth{-1};
    CPX_HANDLE_CALL(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEUID, &nodeuid));
    CPX_HANDLE_CALL(CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODEDEPTH, &nodedepth));

    // Track the highest LB seen at the root; record it on first transition to depth > 0
    if (nodedepth == 0) {
        double best_lb{-1};
        CPX_HANDLE_CALL(CPXcallbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_lb));
        double current = last_root_lb.load();
        while (best_lb > current && !last_root_lb.compare_exchange_weak(current, best_lb)) {
        }
    } else {
        ensure_lb_rootnode(last_root_lb.load());
    }

    std::vector<double> lb(inst.m);
    std::vector<double> ub(inst.m);
    CPXcallbackgetgloballb(context, lb.data(), 0, static_cast<int>(inst.m - 1));
    CPXcallbackgetglobalub(context, ub.data(), 0, static_cast<int>(inst.m - 1));
    unsigned int count_lb = 0;
    unsigned int count_ub = 0;
    for (unsigned int i = 0; i < inst.m; i++) {
        if ((lb[i] > HPLUS_EPSILON) && !inst.fixed_actions[i]) {
            count_lb++;
        }
        if (ub[i] < 1 - HPLUS_EPSILON) {
            count_ub++;
        }
    }
    STATS.series_push<"n_fixed_zero">(count_ub);
    STATS.series_push<"n_fixed_one">(count_lb);

    // If we don't want fractional cuts or we used the custom cutloop and don't want cuts at nodes we can exit
    if (exec.fract_cuts == "0" || (!exec.fract_cuts_at_nodes && exec.custom_cutloop)) {
        return;
    }

    thread_local static std::unordered_set<int> visited_nodes;
    // If we have our custom cutloop in place, we don't need to generate cuts from fractionary solutions in the first root node relaxation
    if (exec.custom_cutloop && nodeuid == 0) {
        return;
    }
    // If we don't want cut at nodes we exit
    if (!exec.fract_cuts_at_nodes && nodedepth != 0) {
        return;
    }
    // If we already called this callback from this node, we skip (unless we are in a root node... in this case we try CPLEX's cutloop)
    if (nodedepth != 0 && visited_nodes.contains(nodeuid)) {
        return;
    }
    visited_nodes.insert(nodeuid);

    STATS.counter_inc<"fract_calls">();
    auto _fract_time = make_scoped_timer<"fract_callback">(STATS);

    std::vector<double> relax_point(inst.m + inst.nfadd);
    double obj{CPX_INFBOUND};
    CPX_HANDLE_CALL(CPXcallbackgetrelaxationpoint(context, relax_point.data(), 0, static_cast<int>(inst.m + inst.nfadd - 1), &obj));

    // Fix numerical errors
    for (auto& val : relax_point) {
        if (val <= HPLUS_EPSILON) {
            val = 0;
        } else if (val >= 1 - HPLUS_EPSILON) {
            val = 1;
        }
    }

    const auto& fadd_weights = relax_cuts::relaxationpoint_info(inst, relax_point);

    // exec.fract_cuts is a string containing one letter per type of cut to be applied
    // -> m : Max-Flow landmark cuts
    // -> l : LMcut landmark cuts
    // -> s : SEC cuts
    try {
        if (exec.fract_cuts.find('m') != std::string::npos || (exec.fract_cuts.find('l') != std::string::npos)) {
            auto fract_lm = relax_cuts::add_lm_cut(context, exec, inst, relax_point);
            STATS.counter_inc<"fract_lm">(fract_lm);
        }
        if (exec.fract_cuts.find('s') != std::string::npos) {
            auto fract_sec = relax_cuts::add_sec_cut(context, inst, fadd_weights);
            STATS.counter_inc<"fract_sec">(fract_sec);
        }
    } catch (timelimit_exception&) {
        return;
    }
}
