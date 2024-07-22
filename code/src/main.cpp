#include "../include/hplus_instance.hpp"

int main() {

    int status = 0;

    Logger logger = Logger("Parser testing", "agricola-opt18-strips-p01.log");
    HPLUS_problem* problem = NULL;
    std::string test_instance = std::string(HPLUS_INST_DIR)+"/agricola-opt18-strips-p01.sas";
    HPLUS_domain* domain = new HPLUS_domain(test_instance, problem, &logger);

    logger.raise_error("Only the parser has been implemented yet, code exits now.");

    delete domain;
    delete problem;

    return status;

}