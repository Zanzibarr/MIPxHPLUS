#include "../include/utils.hpp"
#include "../include/logging.hpp"
#include "../include/hplus_instance.hpp"

int main() {

    int status = 0;

    Logger logger = Logger("Test run 2", "test1.log");
    HPLUS_domain domain = HPLUS_domain("testfile.sas", &logger);

    logger.print_info("Info 1");
    logger.print_warn("Warn 1");
    logger.raise_error("Error 1");
    logger.print_info("Info 2");

    return status;

}