#ifndef _UTILS_H
#define _UTILS_H

#include <iostream>
#include <string>
#include <vector>

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

/**
 * Environment for the execution of the code
 */
struct Environment {

    std::string run_name;
    std::string log_name;

};

/**
 * Statistics for the execution of the code
 */
struct Statistics {

};

/**
BitField used to compactly store size bits into a single data structure
 */
class BitField {

    public:
        /**
         * Constructor of the BitField
         * 
         * @param size: number of bits the BitField has
         */
        BitField(unsigned int size);

        /**
         * Destructor of the BitField
         */
        ~BitField();
        
        /**
         * Set the i th bit to 1
         * 
         * @param i: Index of the bit to set to 1
         */
        void set(const unsigned int i);

        /**
         * Set the i th bit to 0
         * 
         * @param i: Index of the bit to set to 0
         */
        void unset(const unsigned int i);

        /**
         * Access the i th bit
         * 
         * @param i: Index of the bit to access
         * @return The value of the bit in i th position
         */
        bool operator [] (const unsigned int i) const;

        /**
         * Bitwise and of two BitFields
         * 
         * @param bf: BitField to do the and with
         * @return The resulting BitField
         */
        BitField operator & (const BitField bf) const;

        /**
         * Returns the size of the BitField (in bits)
         * 
         * @return The number of bits the BitField contains
         */
        unsigned int size() const;

    private:
        char* field;
        unsigned int len;

};

#endif