#include "../include/hplus_instance.hpp"

int main() {

    int status = 0;

    // TODO : Parse values from command line
    Logger logger = Logger("Parser testing", "agricola-opt18-strips-p01.log");
    HPLUS_problem* problem = nullptr;
    std::string test_instance = std::string(HPLUS_INST_DIR)+"/agricola-opt18-strips-p01.sas";
    HPLUS_domain* domain = new HPLUS_domain(test_instance, problem, &logger);

    logger.print_warn("Only the parser has been implemented.");

    delete problem; problem = nullptr;
    delete domain; domain = nullptr;

    return status;

}