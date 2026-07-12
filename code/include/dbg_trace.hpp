// ~~~~~~~~~~ TEMP DEBUG: candidate/cut tracing for determinism experiments (delete this file when done) ~~~~~~~~~~ //
// Usage: set HPLUS_TRACE=/path/to/trace.txt to enable. One line per event:
//   W <hash> used=<n>                      -> warm start posted to CPLEX (hash of its used-action set)
//   C <hash> cost=<c> used=<n> dec=<F|R|X> tid=<t> -> candidate callback invocation (hash of used-action set)
//   L cand=<hash> cut=<hash> size=<n>      -> landmark cut returned for candidate <hash>
// Compare two runs: `grep '^C' t1 | awk '{print $2,$5}' | sort | uniq -c` vs the same for t2 (candidate multiset),
// and `diff` the sorted L lines (cut multiset). Same-candidate-different-cut across runs = our nondeterminism;
// different candidate multiset with identical responses = CPLEX-side.
// Kill switches (independent of tracing): HPLUS_NO_EARLYEXIT=1 disables the terminate-on-optimality-proof signal,
// HPLUS_NO_POSTSOL=1 disables CPXcallbackpostheursoln in the reject-with-new-solution path.
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace dbg_trace {
inline FILE* file() {
    static FILE* f = [] {
        const char* path = std::getenv("HPLUS_TRACE");
        return path != nullptr ? std::fopen(path, "w") : nullptr;
    }();
    return f;
}
inline thread_local std::uint64_t cur_cand_hash = 0;
inline auto fnv(std::uint64_t h, std::uint64_t v) -> std::uint64_t {
    h ^= v;
    h *= 0x100000001b3ULL;
    return h;
}
}  // namespace dbg_trace
