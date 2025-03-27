#include "rankooh_dynamic.hpp"

void rankooh_dynamic::build_cpx_model([[maybe_unused]] CPXENVptr& cpxenv, [[maybe_unused]] CPXLPptr& cpxlp,
                                      [[maybe_unused]] const hplus::instance& inst, [[maybe_unused]] const hplus::environment& env,
                                      [[maybe_unused]] const logger& log, [[maybe_unused]] hplus::statistics& stats) {
    PRINT_VERBOSE(log, "Building Rankooh's dynamic model.");

    todo(log, "rankooh_dynamic::build_cpx_model");  // TODO
}

void rankooh_dynamic::post_cpx_warmstart([[maybe_unused]] CPXENVptr& cpxenv, [[maybe_unused]] CPXLPptr& cpxlp,
                                         [[maybe_unused]] const hplus::instance& inst, [[maybe_unused]] const hplus::environment& env,
                                         [[maybe_unused]] const logger& log) {
    PRINT_VERBOSE(log, "Posting warm start to Rankooh's dynamic model.");

    todo(log, "rankooh_dynamic::post_cpx_warmstart");  // TODO
}

void rankooh_dynamic::store_cpx_sol([[maybe_unused]] CPXENVptr& cpxenv, [[maybe_unused]] CPXLPptr& cpxlp, [[maybe_unused]] hplus::instance& inst,
                                    [[maybe_unused]] const logger& log) {
    PRINT_VERBOSE(log, "Storing Rankooh's dynamic solution.");

    todo(log, "rankooh_dynamic::store_cpx_sol");  // TODO
}