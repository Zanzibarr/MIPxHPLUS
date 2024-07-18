#include "../include/utils.hpp"

BitField::BitField(unsigned int size) { this -> len = size; this -> field = new char[(size+7)/8](); }

BitField::~BitField() { delete[] this -> field; this -> field = NULL; }

void BitField::set(const unsigned int i) { this -> field[i/8] |= (1 << i%8); }

void BitField::unset(const unsigned int i) { this -> field[i/8] &= ~(1 << i%8); }

bool BitField::operator [] (const unsigned int i) const { return this -> field[i/8] & (1 << i%8); }

BitField BitField::operator & (const BitField bf) const {

    BitField ret = BitField(this -> len);
    for (int i = 0; i < this -> len; i++) if ((this -> field[i/8] & (1 << i%8)) && bf[i]) ret.set(i);
    return ret;

}

unsigned int BitField::size() const { return this -> len; }