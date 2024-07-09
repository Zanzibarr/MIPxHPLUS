#ifndef _UTILS_H
#define _UTILS_H

#include <iostream>
#include <string>

// directory organization
#define HOME_DIR "/Users/matteozanella/Documents/git/thesis_master/code" //TODO: Maybe not hardcoded?
#define LOG_DIR "logs"
#define DEF_LOG_FILE "default.log"

// verbose parameter for logging
#ifndef VERBOSE
#define VERBOSE 0
#endif

// integrity checks switch
#define INTCHECKS VERBOSE>0

#endif