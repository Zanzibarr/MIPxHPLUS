#include "../include/rankooh_model.hpp"

void HPLUS_cpx_build_rankooh(CPXENVptr& env, CPXLPptr& lp, const HPLUS_instance& inst) {

    const auto nvar = inst.get_nvar();
    const auto nact = inst.get_nact();
    const auto nvarstrips = inst.get_nvar_strips();
    const auto& actions = inst.get_actions();
    const auto& variables = inst.get_variables();
    const auto& istate = inst.get_istate();
    const auto& gstate = inst.get_gstate();

    // TODO
    my::todo();

    my::assert(!CPXwriteprob(env, lp, (HPLUS_CPLEX_OUT_DIR"/lp/"+HPLUS_env.run_name+".lp").c_str(), "LP"), "CPXwriteprob failed.");

    HPLUS_env.logger.print_info("Created CPLEX lp for rankooh.");
    
}

void HPLUS_store_rankooh_sol(const CPXENVptr& env, const CPXLPptr& lp, HPLUS_instance& inst) {

    // TODO
    my::todo();

}