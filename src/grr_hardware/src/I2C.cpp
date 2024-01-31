#include "bcm2835.h"
#include <iostream>
#include "../include/I2C.h"

#ifndef I2CCPP
#define I2CCPP
/*! Reads data from reg on addr to dst
        \param[in] addr The I2C slave Addres
        \param[in] reg The register to begin reading from
        \param[in] n the number of bytes to read
        \param[out] dst a pointer that will be filled with the bytes
        \return true if sucessful, False on error
*/
bool I2C::readRegisters(const uint8_t addr, const uint8_t reg, uint8_t n, uint8_t* dst)
{
    std::cout << "Begin Read from " << std::to_string(addr) << " reg: " << std::to_string(reg) << " with " << std::to_string(n) << " bytes" << std::endl;
    bcm2835_i2c_setSlaveAddress(addr);
    char buf[1] = {(char)(10000000 | reg)};
    std::cout << "buf" << (int) buf[0] << std::endl;
    uint8_t reason1 = bcm2835_i2c_write(buf, sizeof(buf));
    char buff[n];
    for (int i=0; i<n; i++) buff[i] = 'n';
    uint8_t reason2 = bcm2835_i2c_read(buff, n);
    dst = (uint8_t*) &buff;

    std::cout << "Reason 1: " << std::to_string(reason1) << " Reason 2: " << std::to_string(reason2) << std::endl;

    return (reason1 == BCM2835_I2C_REASON_OK) && (reason2 == BCM2835_I2C_REASON_OK);
}

/*! writes data to reg on addr from src
        \param[in] addr The I2C slave Addres
        \param[in] reg The register to begin writing to
        \param[in] src a pointer to the bytes to write
        \param[in] n the number of bytes to write
        \return True if sucessful, False on error
*/
bool I2C::writeRegisters(const uint8_t addr, const uint8_t reg, const uint8_t *src, const uint8_t n)
{
    std::cout << "write reg" << std::endl;
    std::cout << "writing " << std::to_string(n) << " Bytes to reg " << std::to_string(reg) << " at addy " << std::to_string(addr) << std::endl;
    bcm2835_i2c_setSlaveAddress(addr);
    char buf[n + 1];
    buf[0] = reg;
    for (int i = 1; i <= n + 1; i++)
    {
        buf[i] = src[i - 1];
    }
    uint8_t reason = bcm2835_i2c_write(buf, n + 1);
    std::cout << std::to_string(reason) << std::endl;
    return reason == BCM2835_I2C_REASON_OK;    
    
}

int main(){
    bcm2835_i2c_setSlaveAddress(0x29);
    char buf[1] = {(char)(10010100)};
    uint8_t reason1 = bcm2835_i2c_write(buf, sizeof(buf));
}

#endif