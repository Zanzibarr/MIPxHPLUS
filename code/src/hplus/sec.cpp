#include "solver.hpp"

// SEC(Pi) separator on integer solutions (Eq. (12), Section 4.1) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
void Solver::hplus_separate_cand_sec_cut_(CPXCALLBACKCONTEXTptr /*context*/) {
    // TODO
    unimplemented();
}

void Solver::hplus_reject_cand_sec_(CPXCALLBACKCONTEXTptr /*context*/, const std::vector<std::vector<unsigned int>>& /*cycles*/) {
    // TODO
    unimplemented();
}

// fSEC(Pi) separator on fractional solutions (Algorithm 3, Section 6.1) - Salvagnin, Zanella: "MIP Formulations for Delete-Free AI Planning"
void Solver::hplus_separate_relax_sec_cut_(CPXCALLBACKCONTEXTptr /*context*/) {
    // TODO
    unimplemented();
}

void Solver::hplus_reject_relax_sec_(CPXCALLBACKCONTEXTptr /*context*/, const std::vector<std::vector<unsigned int>>& /*cycles*/) {
    // TODO
    unimplemented();
}
