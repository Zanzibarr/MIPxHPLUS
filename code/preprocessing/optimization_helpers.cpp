#include "preprocessing.hpp"

void prep::eliminated_facts_removal(hplus::instance& inst, std::vector<std::vector<unsigned int>>& landmarks) {
    unsigned int removed{static_cast<unsigned int>(inst.eliminated_facts.sparse().size())};

    std::vector<unsigned int> removed_offsets(inst.n, 0);
    unsigned int current_removed = 0;
    for (unsigned int i = 0; i < inst.n; i++) {
        removed_offsets[i] = current_removed;
        if (inst.eliminated_facts[i]) {
            ++current_removed;
        }
    }

    for (unsigned int act_i = 0; act_i < inst.m; act_i++) {
        std::vector<unsigned int> new_pre;
        std::vector<unsigned int> new_eff;
        new_pre.reserve(inst.actions[act_i].pre_sparse.size());
        new_eff.reserve(inst.actions[act_i].eff_sparse.size());
        for (const auto& pre : inst.actions[act_i].pre_sparse) {
            if (!inst.eliminated_facts[pre]) {
                new_pre.push_back(pre - removed_offsets[pre]);
            }
        }
        for (const auto& eff : inst.actions[act_i].eff_sparse) {
            if (!inst.eliminated_facts[eff]) {
                new_eff.push_back(eff - removed_offsets[eff]);
            }
        }
        inst.actions[act_i].pre_sparse = std::move(new_pre);
        inst.actions[act_i].eff_sparse = std::move(new_eff);
    }

    // Adjust goal
    BinarySet new_goal(inst.n - removed);
    for (const auto p : inst.goal) {
        if (!inst.eliminated_facts[p]) {
            new_goal.add(p - removed_offsets[p]);
        }
    }
    inst.goal = new_goal;

    // Adjust landmarks
    unsigned int counter{0};
    for (const auto fact : !inst.eliminated_facts) {
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

    // Adjust fixed_facts
    BinarySet new_fixed_facts(inst.n - removed);
    for (const auto fact : inst.fixed_facts) {
        if (!inst.eliminated_facts[fact]) {
            new_fixed_facts.add(fact - removed_offsets[fact]);
        }
    }
    inst.fixed_facts = new_fixed_facts;

    inst.n -= removed;
    STATS.counter_set<"n_prep">(static_cast<int64_t>(inst.n - inst.fixed_facts.sparse().size()));
}

void prep::eliminated_actions_removal(hplus::instance& inst) {
    BinarySet new_fixed_actions(static_cast<unsigned int>(inst.m - inst.eliminated_actions.sparse().size()));
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
    STATS.counter_set<"m_prep">(static_cast<int64_t>(inst.m - inst.fixed_actions.sparse().size()));
    unsigned int count{0};
    for (const auto& act : inst.actions) {
        count += act.eff_sparse.size();
    }
    inst.nfadd = count;
    STATS.counter_set<"nfadd_prep">(count);
}