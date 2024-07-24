#include "../include/hplus_instance.hpp"

void start() {

    my::start_timer();
    HPLUS_env.status = 0;
    HPLUS_env.logger = Logger("Parser testing", "agricola-opt18-strips-p01.log");

}

void end() {

    HPLUS_stats.print();

}

int main() {

    start();

    // TODO : Parse values from command line

    std::string test_instance = std::string(HPLUS_INST_DIR)+"/agricola-opt18-strips-p01.sas";
    HPLUS_instance* inst = new HPLUS_instance(test_instance);

    HPLUS_env.logger.print_warn("Only the parser has been implemented.");

    DEL(inst);

    end();

    return HPLUS_env.status;

}