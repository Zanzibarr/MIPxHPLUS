#pragma once

#include <array>
#include <ct_string.hxx>
#include <string>
#include <string_view>

// Metadata for the CLI arguments registered in cli_parser: parameter names,
// help descriptions and lists of allowed values.
namespace cli_desc {
using utilz::CTString;

// ── Parameter names (compile-time, usable as ArgParser NTTP keys) ──
inline constexpr CTString verbose = "verbose";
inline constexpr CTString seed = "seed";
inline constexpr CTString stdout = "stdout";
inline constexpr CTString sol_file = "solfile";
inline constexpr CTString log = "log";
inline constexpr CTString cpxlog = "cpxlog";
inline constexpr CTString input = "input";
inline constexpr CTString time_limit = "time_limit";
inline constexpr CTString threads = "threads";
inline constexpr CTString mode = "mode";
inline constexpr CTString hplus_alg = "hplus-alg";
inline constexpr CTString preprocess = "preprocess";
inline constexpr CTString lmcut_pcf = "lmcut-pcf";
inline constexpr CTString lmcut_min = "lmcut-min";
inline constexpr CTString lmcut_opt = "lmcut-opt";
inline constexpr CTString primal_heur = "primal-heur";
inline constexpr CTString cand_cuts = "cand-cuts";
inline constexpr CTString relax_cuts = "relax-cuts";

// --- Defaults ---
// General parameters
constexpr bool def_debug_enabled = false;
constexpr std::string def_sol_file = "0";

// Execution parameters
constexpr int def_time_limit_s = 60;
constexpr int def_threads = 16;
constexpr bool def_stdout = true;
constexpr std::string def_log = "0";
constexpr std::string def_cpxlog = "cplex.log";
constexpr int def_seed = 2122187;
constexpr std::string def_mode = "hplus";
constexpr std::string def_hplus_alg = "base";

// Preprocessing
constexpr std::string def_preprocess = "lafbdi";

// LMCut
constexpr std::string def_lmcut_pcf = "aiv";
constexpr std::string def_lmcut_min = "c";
constexpr std::string def_lmcut_opt = "ef";

// Primal Heuristic
constexpr std::string def_primal_heur = "gha";

// Cut separators
constexpr std::string def_cand_cuts = "lmcut-c";
constexpr std::string def_relax_cuts = "lmcut-g";

// ── Help descriptions ──
inline constexpr std::string_view verbose_help = "Enable verbose (DEBUG-level) logging";
inline constexpr std::string_view sol_file_help = "Write the solution to file (Fast Downward format)";
inline constexpr std::string_view seed_help = "Random seed for reproducible runs";
inline constexpr std::string_view stdout_help = "Also write output to stdout (independent of --log)";
inline constexpr std::string_view log_help = "Path to the solver log file, or 0 for none (parent directory must exist)";
inline constexpr std::string_view cpxlog_help = "Path to CPLEX's log file, or 0 for none (parent directory must exist)";
inline constexpr std::string_view input_help = "Path to the SAS+ task to solve (must exist)";
inline constexpr std::string_view time_limit_help = "Wall-clock time limit in seconds; 0 means no limit";
inline constexpr std::string_view threads_help = "Max threads to use; capped to the machine's core count";
inline constexpr std::string_view mode_help =
    "What to compute: info (task statistics only), lmcut (LM-Cut bound), hplus-feas (a feasible relaxed plan), hplus (an optimal relaxed plan). "
    "Only hplus uses --hplus-alg; info/lmcut disable --primal-heur, and info also disables --preprocess and --lmcut-pcf";
inline constexpr std::string_view hplus_alg_help =
    "MIP formulation for the hplus problem (base, tl, ve); used only when --mode=hplus. Candidate cuts (--cand-cuts) are required for the base model";
inline constexpr std::string_view preprocess_help =
    "Preprocessing to apply as a combo of letters (0 for none): l (landmarks extraction), a (first adders, requires l), f (forward relevance "
    "analysis, requires b), b (backward relevance analysis), d (dominated actions removal, requires l), o (orbital probing), i (initial actions "
    "sequencing), g (fixed facts get promoted to goal facts), p (non-goal facts that are not preconditions get eliminated)";
inline constexpr std::string_view lmcut_pcf_help =
    "Precondition Choice Function(s) driving LM-Cut as a combo of letters (0 to skip LM-Cut): a (ARB), i (INV), v (VDM), r (RND), z (GZD), b (BD), "
    "d (GZD+BD); each extra letter runs LM-Cut once more";
inline constexpr std::string_view lmcut_min_help =
    "Landmark minimization inside LM-Cut: g (greedy), c (complete), 0 for none; forced to 0 when --lmcut-pcf=0";
inline constexpr std::string_view lmcut_opt_help =
    "Landmark optimizations inside LM-Cut as a combo of letters: 0 (None), e (non goal-section effects of an action crossing the cut are not added "
    "to the pre-goal), f (fixed actions removed from LMCut computation)";
inline constexpr std::string_view primal_heur_help =
    "Primal heuristic used to warm-start the MIP or for the hplus-feas mode (0 for none); disabled in info/lmcut modes";
inline constexpr std::string_view cand_cuts_help =
    "Cut settings for the candidate (integer-solution) separator; 0: none, sec: Subtour Elimination Constraints, lm-f: Frontier Landmarks, lm-c: "
    "Complementary Minimal Landmarks, lmcut: LM-Cut separator, lmcut-g/-c: Minimal LM-Cut";
inline constexpr std::string_view relax_cuts_help =
    "Cut settings for the relaxation (fractional-solution) separator; 0: none, sec: Subtour Elimination Constraints, lm: Max-Flow Landmarks, lm-m: "
    "Minimal Max-Flow Landmarks, lmcut: LM-Cut separator, lmcut-g/-c: Minimal LM-Cut";

// ── Allowed values ──
inline constexpr std::array<std::string_view, 4> mode_choices = {"info", "lmcut", "hplus-feas", "hplus"};
inline constexpr std::array<std::string_view, 4> hplus_alg_choices = {"0", "base", "tl", "ve"};
inline constexpr std::array<std::string_view, 3> lmcut_min_choices = {"0", "g", "c"};
inline constexpr std::array<std::string_view, 5> primal_heur_choices = {"0", "gc", "gcxe", "gha", "ghm"};
inline constexpr std::array<std::string_view, 7> cand_cuts_choices = {"0", "sec", "lm-f", "lm-c", "lmcut", "lmcut-g", "lmcut-c"};
inline constexpr std::array<std::string_view, 7> relax_cuts_choices = {"0", "sec", "lm", "lm-m", "lmcut", "lmcut-g", "lmcut-c"};

// Letter-combination allow lists (validated by sanitize_letters, not by ArgParser::allow)
inline constexpr std::string_view preprocess_letters = "lafbdoigp";
inline constexpr std::string_view lmcut_pcf_letters = "aivrzbd";
inline constexpr std::string_view lmcut_opt_letters = "ef";
}  // namespace cli_desc
