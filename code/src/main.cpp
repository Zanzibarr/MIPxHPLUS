#include "../include/utils.hpp"
#include "../include/hplus_instance.hpp"

int main() {

    int status = 0;

    Logger logger = Logger("Test run", "test.log");
    HPLUS_problem* problem = NULL;
    HPLUS_domain* domain = new HPLUS_domain("/Users/matteozanella/Documents/git/thesis_master/local/test.sas", problem, &logger);

    logger.raise_error("Only the parser has been implemented yet, code exits now.");

    delete domain;
    delete problem;

    return status;

}