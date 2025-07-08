#include "exact.hpp"

void cutloop::cutloop([[maybe_unused]] CPXENVptr& env, [[maybe_unused]] CPXLPptr& lp, [[maybe_unused]] const hplus::execution& exec,
                      [[maybe_unused]] const hplus::instance& inst, [[maybe_unused]] const hplus::statistics& stats) {
    LOG_WARNING << "Custom cutloop has yet to be implemented.";
}