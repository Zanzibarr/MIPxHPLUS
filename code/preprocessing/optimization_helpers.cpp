#include "preprocessing.hpp"

void prep::eliminated_facts_removal(hplus::instance& inst, hplus::statistics& stats, std::vector<std::vector<unsigned int>>& landmarks) {
    // Remove eliminated facts from preconditions and effects
    unsigned int removed{static_cast<unsigned int>(inst.eliminated_facts.sparse().size())};
    std::vector<unsigned int> removed_offsets(inst.n, 0);
    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        binary_set new_pre(inst.n - removed);
        binary_set new_eff(inst.n - removed);
        unsigned int current_removed{0};
        for (unsigned int i = 0; i < inst.n; i++) {
            removed_offsets[i] = current_removed;
            if (!inst.eliminated_facts[i]) {
                if (inst.actions[act_i].pre[i]) new_pre.add(i - current_removed);
                if (inst.actions[act_i].eff[i]) new_eff.add(i - current_removed);
            } else {
                current_removed++;
            }
        }
        inst.actions[act_i].pre = new_pre;
        inst.actions[act_i].eff = new_eff;
        inst.actions[act_i].pre_sparse = new_pre.sparse();
        inst.actions[act_i].eff_sparse = new_eff.sparse();
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
    for (const auto& p : !inst.eliminated_facts) {
        std::vector<unsigned int> copy(landmarks[p]);
        landmarks[counter].clear();
        for (const auto& l : copy) {
            if (inst.eliminated_facts[l]) continue;
            landmarks[counter].push_back(l - removed_offsets[l]);
        }
        counter++;
    }
    landmarks.resize(counter);

    // Adjust positions of facts in fixed_facts
    binary_set new_fixed_facts(inst.n - removed);
    for (const auto& p : inst.fixed_facts) {
        if (!inst.eliminated_facts[p]) {
            new_fixed_facts.add(p - removed_offsets[p]);
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
                if (inst.fixed_actions[read_pos]) new_fixed_actions.add(write_pos);
            }
            ++write_pos;
        }
    }
    inst.actions.resize(write_pos);
    inst.actions_names.resize(write_pos);
    inst.fixed_actions = new_fixed_actions;

    inst.m = static_cast<unsigned int>(write_pos);
    stats.m_prep = inst.m - inst.fixed_actions.sparse().size();
    unsigned int count{0};
    for (const auto& a : inst.actions) count += a.eff_sparse.size();
    inst.nfadd = count;
    stats.nfadd_prep = count;
}