#include "algorithms.hpp"
#include "preprocessing.hpp"

void prep::eliminated_facts_removal(hplus::instance& inst, hplus::statistics& stats, std::vector<std::vector<unsigned int>>& landmarks) {
    // Remove eliminated facts from preconditions and effects
    unsigned int removed{static_cast<unsigned int>(inst.eliminated_facts.sparse().size())};
    std::vector<unsigned int> removed_offsets(inst.n, 0);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        std::vector<unsigned int> new_pre;
        std::vector<unsigned int> new_eff;
        unsigned int current_removed{0};
        for (unsigned int i = 0; i < inst.n; i++) {
            removed_offsets[i] = current_removed;
            if (!inst.eliminated_facts[i]) {
                if (sorted_contains(inst.actions[act_i].pre_sparse, i)) {
                    new_pre.push_back(i - current_removed);
                }
                if (sorted_contains(inst.actions[act_i].eff_sparse, i)) {
                    new_eff.push_back(i - current_removed);
                }
            } else {
                current_removed++;
            }
        }
        inst.actions[act_i].pre_sparse = new_pre;
        inst.actions[act_i].eff_sparse = new_eff;
    }

    // Adjust positions of facts in the goal
    binary_set new_goal(inst.n - removed);
    for (const auto& p : inst.goal) {
        if (!inst.eliminated_facts[p]) {
            new_goal.add(p - removed_offsets[p]);
        }
    }
    inst.goal = new_goal;

    // Adjust positions of facts in landmarks
    unsigned int counter{0};
    for (const auto& fact : !inst.eliminated_facts) {
        std::vector<unsigned int> copy(landmarks[fact]);
        landmarks[counter].clear();
        for (const auto& l : copy) {
            if (inst.eliminated_facts[l]) {
                continue;
            }
            landmarks[counter].push_back(l - removed_offsets[l]);
        }
        counter++;
    }
    landmarks.resize(counter);

    // Adjust positions of facts in fixed_facts
    binary_set new_fixed_facts(inst.n - removed);
    for (const auto& fact : inst.fixed_facts) {
        if (!inst.eliminated_facts[fact]) {
            new_fixed_facts.add(fact - removed_offsets[fact]);
        }
    }
    inst.fixed_facts = new_fixed_facts;

    inst.n -= removed;
    stats.n_prep = inst.n - inst.fixed_facts.sparse().size();
}

void prep::eliminated_actions_removal(hplus::instance& inst, hplus::statistics& stats) {
    binary_set new_fixed_actions(inst.m - inst.eliminated_actions.sparse().size());
    unsigned int write_pos = 0;
    for (unsigned int read_pos = 0; read_pos < inst.actions.size(); ++read_pos) {
        if (!inst.eliminated_actions[read_pos]) {
            if (write_pos != read_pos) {
                // Remove eliminated actions
                inst.actions[write_pos] = std::move(inst.actions[read_pos]);
                // Remove eliminated action names
                inst.actions_names[write_pos] = std::move(inst.actions_names[read_pos]);
                // Adjust positions of actions in fixed_actions
                if (inst.fixed_actions[read_pos]) {
                    new_fixed_actions.add(write_pos);
                }
            }
            ++write_pos;
        }
    }
    inst.actions.resize(write_pos);
    inst.actions_names.resize(write_pos);
    inst.fixed_actions = new_fixed_actions;

    inst.m = write_pos;
    stats.m_prep = inst.m - inst.fixed_actions.sparse().size();
    unsigned int count{0};
    for (const auto& act : inst.actions) {
        count += act.eff_sparse.size();
    }
    inst.nfadd = count;
    stats.nfadd_prep = count;
}