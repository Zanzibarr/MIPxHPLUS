#include "../include/utils.hpp"
#include "../include/logging.hpp"
#include "../include/hplus_instance.hpp"

int main() {

    int status = 0;

    Logger logger = Logger("Test run", "test.log");
    HPLUS_domain domain = HPLUS_domain("testfile.sas", &logger);

    logger.raise_error("Test error.");
    logger.print_info("This shouldn't appear.");

    return status;

}