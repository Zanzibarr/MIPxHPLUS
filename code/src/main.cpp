#include "../include/hplus_instance.hpp"

int main() {

    int status = 0;

    // TODO : Parse values from command line
    Logger logger = Logger("Parser testing", "agricola-opt18-strips-p01.log");
    std::string test_instance = std::string(HPLUS_INST_DIR)+"/agricola-opt18-strips-p01.sas";
    HPLUS_instance* inst = new HPLUS_instance(test_instance, &logger);

    logger.print_warn("Only the parser has been implemented.");

    delete inst; inst = nullptr;

    return status;

}