#include "cli_descriptions.hpp"
#include "solver.hpp"
#include "utils.hpp"

void Solver::solve_primal_() {
    const auto& primal = params_.get<cli_desc::primal_heur, std::string>();

    if (primal == "gc" || primal == "gcxe" || primal == "gha" || primal == "ghm") {
        primal_greedy_();
    } else {
        logger_[WARNING] << std::format("Unhandled primal algorithm: {}", primal);
    }

    if (is_gr_or_eq_double(global_.best_bound, global_.best_incumbent)) {
        throw EarlyExit("computing the primal heuristic", EarlyExit::OPTIMAL);
    }
}