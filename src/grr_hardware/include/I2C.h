#include <iostream>

#ifndef I2CH
#define I2CH

class I2C{
    public:
        static bool readRegisters(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t *const dst);
        static bool writeRegisters(const uint8_t addr, const uint8_t reg, const uint8_t *const src, const uint8_t n);
};

#endif