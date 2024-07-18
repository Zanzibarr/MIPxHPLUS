#include "../headers/utils.hpp"
#include "../headers/logging.hpp"
#include "../headers/hplus_instance.hpp"

int main() {

    int status = 0;

    Logger logger = Logger("Test run 2", "test1.log");

    logger.print_info("Info 1");
    logger.print_warn("Warn 1");
    logger.raise_error("Error 1");
    logger.print_info("Info 2");

    return status;

}