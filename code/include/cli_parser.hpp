#pragma once

#include <argparser.hxx>
#include <cassert>
#include <filesystem>
#include <limits>
#include <logger.hxx>

#include "cli_descriptions.hpp"
#include "utils.hpp"

namespace {
std::vector<std::string> parsing_warnings;

// feed an allowed-values list (stored as an array in cli_descriptions.hpp) into an argument builder
template <typename Arg, std::size_t N>
inline auto apply_allow(Arg& arg, const std::array<std::string_view, N>& choices) -> Arg& {
    for (const auto& choice : choices) {
        arg.allow({std::string{choice}});
    }
    return arg;
}

// letter-combination params: warn about and remove invalid letters; if a 0 is present among other letters, warn and keep only the 0
inline auto sanitize_letters(std::string_view flag, const std::string& value, std::string_view valid) -> std::string {
    std::string kept;
    bool has_zero = false;
    for (const char letter : value) {
        if (letter == '0') {
            has_zero = true;
        } else if (valid.find(letter) != std::string_view::npos) {
            kept += letter;
        } else {
            parsing_warnings.push_back(std::format("Warning: --{}: invalid option '{}' (valid: 0{}), ignoring it\n", flag, letter, valid));
        }
    }
    if (has_zero && !kept.empty()) {
        parsing_warnings.push_back(std::format("Warning: --{}: '0' specified among other options, ignoring the others\n", flag));
    }
    return (has_zero || kept.empty()) ? "0" : kept;
}

inline auto validate_cli(cli::ArgParser& parser) -> void {
    // thread count shouldn't exceed hardware concurrency
    int max_threads = static_cast<int>(std::thread::hardware_concurrency());
    if (parser.get<cli_desc::threads, int>() > max_threads) {
        parser.set<cli_desc::threads>(max_threads);
        parsing_warnings.push_back(std::format("This machine has {} cores: using up to {} threads", max_threads, max_threads));
    }

    // log paths should exist
    auto check_log_dir = [](std::string_view flag, const std::string& path) {
        if (path == "0") {
            return;  // no log file
        }
        auto parent = std::filesystem::path(path).parent_path();
        if (!parent.empty() && !std::filesystem::is_directory(parent)) {
            throw cli::ParseError(std::format("--{}: directory '{}' does not exist", flag, parent.string()));
        }
    };
    check_log_dir(cli_desc::log, parser.get<cli_desc::log, std::string>());
    check_log_dir(cli_desc::cpxlog, parser.get<cli_desc::cpxlog, std::string>());

    // input file should exist
    const auto& input = parser.get<cli_desc::input, std::string>();
    {
        std::ifstream file(input.c_str(), std::ifstream::in);
        if (!file.good()) {
            throw cli::ParseError(std::format("--input: file {} does not exist", input));
        }
    }

    // a time-limit of 0 means numeric_limits<int>::max() as time limit
    if (parser.get<cli_desc::time_limit, int>() == 0) {
        parser.set<cli_desc::time_limit>(std::numeric_limits<int>::max());
    }

    // if mode is not hplus, hplus-alg should be 0
    if (parser.get<cli_desc::mode, std::string>() != "hplus") {
        parser.set<cli_desc::hplus_alg>("0");
    } else {
        if ((parser.get<cli_desc::hplus_alg, std::string>()) == "0") {
            throw cli::ParseError(std::format("{} cannot be '0' when {} is 'hplus'", cli_desc::mode.view(), cli_desc::hplus_alg.view()));
            parser.set<cli_desc::hplus_alg>(cli_desc::def_hplus_alg);
        }
    }
    if (parser.get<cli_desc::mode, std::string>() == "info") {
        parser.set<cli_desc::preprocess>("0");
        parser.set<cli_desc::lmcut_pcf>("0");
        parser.set<cli_desc::lmcut_min>("0");
        parser.set<cli_desc::lmcut_opt>("0");
        parser.set<cli_desc::primal_heur>("0");
        parser.set<cli_desc::hplus_alg>("0");
        parser.set<cli_desc::cand_cuts>("0");
        parser.set<cli_desc::relax_cuts>("0");
    }
    if (parser.get<cli_desc::mode, std::string>() == "lmcut") {
        parser.set<cli_desc::primal_heur>("0");
        parser.set<cli_desc::hplus_alg>("0");
        parser.set<cli_desc::cand_cuts>("0");
        parser.set<cli_desc::relax_cuts>("0");
    }

    {
        std::string choices = sanitize_letters(cli_desc::preprocess, parser.get<cli_desc::preprocess, std::string>(), cli_desc::preprocess_letters);
        if (choices.find('l') == std::string::npos) {
            bool add_l = false;
            if (choices.find('a') != std::string::npos) {
                parsing_warnings.emplace_back("First adders extraction (a) requires landmarks extraction (l): enabling landmarks extraction");
                add_l = true;
            }
            if (choices.find('d') != std::string::npos) {
                parsing_warnings.emplace_back("Dominated actions removal (d) requires landmarks extraction (l): enabling landmarks extraction");
                add_l = true;
            }
            if (add_l) {
                choices += 'l';
            }
        }
        if (choices.find('f') != std::string::npos && choices.find('b') == std::string::npos) {
            parsing_warnings.emplace_back(
                "Forward relevance analysis (f) requires backward relevance analysis (b): enabling backward relevance analysis");
            choices += 'b';
        }
        parser.set<cli_desc::preprocess>(choices);
    }
    parser.set<cli_desc::lmcut_pcf>(
        sanitize_letters(cli_desc::lmcut_pcf, parser.get<cli_desc::lmcut_pcf, std::string>(), cli_desc::lmcut_pcf_letters));

    // if LM-Cut is not executed, no minimization procedure is used
    if (parser.get<cli_desc::lmcut_pcf, std::string>() == "0") {
        parser.set<cli_desc::lmcut_min>("0");
        parser.set<cli_desc::lmcut_opt>("0");
    }

    // Candidate cuts are needed only for base model
    const auto& alg = parser.get<cli_desc::hplus_alg, std::string>();
    if (alg != "base") {
        parsing_warnings.emplace_back(std::format("Candidate callbacks are not needed with the {} formulation: removing them", alg));
        parser.set<cli_desc::cand_cuts>("0");
    } else {
        const auto& cand = parser.get<cli_desc::cand_cuts, std::string>();
        if (cand == "0") {
            throw cli::ParseError("'base' formulation needs candidate callbacks enabled");
        }
    }
}
}  // namespace
inline auto parse_cli(const int argc, char** argv, Logger& logger) -> ParameterRegistry {
    cli::ArgParser parser(argv[0], "Find a solution / the optimal solution to the deletefree relaxation of a SAS+ planning task.");

    // General parameters
    parser.add<cli_desc::deterministic, bool>().shorthand('d').description(cli_desc::deterministic_help).default_val(cli_desc::def_deterministic);
    parser.add<cli_desc::verbose, bool>().shorthand('v').description(cli_desc::verbose_help).default_val(cli_desc::def_debug_enabled);

    // Execution parameters
    parser.add<cli_desc::seed, int>().shorthand('s').description(cli_desc::seed_help).default_val(cli_desc::def_seed);
    parser.add<cli_desc::stdout_, bool>().shorthand('o').description(cli_desc::stdout_help).default_val(cli_desc::def_stdout);
    parser.add<cli_desc::log, std::string>().shorthand('l').description(cli_desc::log_help).default_val(cli_desc::def_log);
    parser.add<cli_desc::cpxlog, std::string>().shorthand('c').description(cli_desc::cpxlog_help).default_val(cli_desc::def_cpxlog);
    parser.add<cli_desc::input, std::string>().shorthand('i').description(cli_desc::input_help).require();
    parser.add<cli_desc::time_limit, int>().shorthand('t').description(cli_desc::time_limit_help).default_val(cli_desc::def_time_limit_s).min(0);
    parser.add<cli_desc::threads, int>().shorthand('T').description(cli_desc::threads_help).default_val(cli_desc::def_threads).min(1);
    apply_allow(parser.add<cli_desc::mode, std::string>().shorthand('m').description(cli_desc::mode_help).default_val(cli_desc::def_mode),
                cli_desc::mode_choices);
    apply_allow(
        parser.add<cli_desc::hplus_alg, std::string>().shorthand('a').description(cli_desc::hplus_alg_help).default_val(cli_desc::def_hplus_alg),
        cli_desc::hplus_alg_choices);

    // Preprocessing
    parser.add<cli_desc::preprocess, std::string>().shorthand('p').description(cli_desc::preprocess_help).default_val(cli_desc::def_preprocess);

    // LM-Cut
    parser.add<cli_desc::lmcut_pcf, std::string>().shorthand('L').description(cli_desc::lmcut_pcf_help).default_val(cli_desc::def_lmcut_pcf);
    apply_allow(
        parser.add<cli_desc::lmcut_min, std::string>().shorthand('M').description(cli_desc::lmcut_min_help).default_val(cli_desc::def_lmcut_min),
        cli_desc::lmcut_min_choices);
    apply_allow(
        parser.add<cli_desc::lmcut_opt, std::string>().shorthand('e').description(cli_desc::lmcut_opt_help).default_val(cli_desc::def_lmcut_opt),
        cli_desc::lmcut_opt_choices);

    // Primal Heuristic
    apply_allow(parser.add<cli_desc::primal_heur, std::string>()
                    .shorthand('H')
                    .description(cli_desc::primal_heur_help)
                    .default_val(cli_desc::def_primal_heur),
                cli_desc::primal_heur_choices);

    // Cut separators
    apply_allow(
        parser.add<cli_desc::cand_cuts, std::string>().shorthand('I').description(cli_desc::cand_cuts_help).default_val(cli_desc::def_cand_cuts),
        cli_desc::cand_cuts_choices);
    apply_allow(
        parser.add<cli_desc::relax_cuts, std::string>().shorthand('F').description(cli_desc::relax_cuts_help).default_val(cli_desc::def_relax_cuts),
        cli_desc::relax_cuts_choices);

    parsing_warnings.clear();
    if (!argparser_parse(parser, argc, argv, validate_cli)) {
        std::exit(EXIT_FAILURE);
    }

    logger.set_memory(parser.get<cli_desc::verbose, bool>());
    logger.set_thread(parser.get<cli_desc::verbose, bool>());
    logger.set_min_level(parser.get<cli_desc::verbose, bool>() ? DEBUG : INFO);
    logger.set_stdout(parser.get<cli_desc::stdout_, bool>());
    if (const auto& log_path = parser.get<cli_desc::log, std::string>(); log_path != "0") {
        logger.open_file(log_path);
    }

    for (auto& warn : parsing_warnings) {
        logger[WARNING] << warn;
    }

    return parser.parameters();
}
