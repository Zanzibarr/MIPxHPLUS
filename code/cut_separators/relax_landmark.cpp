#include "fract_separators.hpp"
#include "relax_callback.hpp"

[[nodiscard]]
auto relax_cuts::add_lm_cut(CPXCALLBACKCONTEXTptr context, const hplus::execution& exec, const hplus::instance& inst,
                            const std::vector<double>& relax_point) -> unsigned int {
    std::vector<std::vector<unsigned int>> landmarks;
    if (exec.fract_cuts.find('m') != std::string::npos) {
        const auto& [found, landmark] = fract_lm_sep::get_r3_violated_landmark(exec, inst, relax_point);
        if (found) {
            landmarks.push_back(landmark);
        }
    }
    if (exec.fract_cuts.find('l') != std::string::npos) {
        const auto& [found, landmarks_lmcut] = fract_lm_sep::get_lmcut_violated_landmarks(exec, inst, relax_point);
        if (found) {
            for (auto landmark : landmarks_lmcut) {
                landmarks.push_back(std::move(landmark));
            }
        }
    }
    for (const auto& landmark : landmarks) {
        std::vector<int> ind(landmark.size());
        unsigned int nnz{0};
        for (const auto& act_i : landmark) {
            ind[nnz++] = static_cast<int>(act_i);
        }
        std::vector<double> val(landmark.size(), 1.0);
        constexpr double rhs{1};
        constexpr char sense{'G'};
        constexpr int begin{0};
        constexpr int purgeable{CPX_USECUT_FORCE};
        constexpr int local{0};
        CPX_HANDLE_CALL(CPXcallbackaddusercuts(context, 1, static_cast<int>(nnz), &rhs, &sense, &begin, ind.data(), val.data(), &purgeable, &local));
    }
    return static_cast<unsigned int>(landmarks.size());
}
