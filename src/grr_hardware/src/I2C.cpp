#include "bcm2835.h"
#include <iostream>
#include "I2C.h"

/*! Reads data from reg on addr to dst
        \param[in] addr The I2C slave Addres
        \param[in] reg The register to begin reading from
        \param[in] n the number of bytes to read
        \param[out] dst a pointer that will be filled with the bytes
        \return true if sucessful, False on error
*/
bool I2C::readRegisters(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t *const dst)
{
    bcm2835_i2c_setSlaveAddress(addr);
    char buf[1] = {reg};
    uint8_t reason1 = bcm2835_i2c_write(buf, 1);
    uint8_t reason2 = bcm2835_i2c_read((char *)dst, n);
    return (reason1 == BCM2835_I2C_REASON_OK) && (reason2 == BCM2835_I2C_REASON_OK);
}

/*! writes data to reg on addr from src
        \param[in] addr The I2C slave Addres
        \param[in] reg The register to begin writing to
        \param[in] src a pointer to the bytes to write
        \param[in] n the number of bytes to write
        \return True if sucessful, False on error
*/
bool I2C::writeRegisters(const uint8_t addr, const uint8_t reg, const uint8_t *const src, const uint8_t n)
{
    // std::cout << "writing " << std::to_string(n) << " Bytes to " << std::to_string(reg) << std::endl;
    bcm2835_i2c_setSlaveAddress(addr);
    char buf[n + 1];
    buf[0] = reg;
    for (int i = 1; i <= n + 1; i++)
    {
        buf[i] = src[i - 1];
    }
    uint8_t reason = bcm2835_i2c_write(buf, n + 1);
    return reason == BCM2835_I2C_REASON_OK;
}