#include "backcolorsensor.h"

#define TCS34725_ENABLE     (0x00)    /**< Interrupt Enable register */
#define TCS34725_ENABLE_PON (0x01)    /**< Power on - Writing 1 activates the internal oscillator, 0 disables it */
#define TCS34725_ENABLE_AEN (0x02)    /**< RGBC Enable - Writing 1 actives the ADC, 0 disables it */

#define TCS34725_CDATAL (0x14) /**< Clear channel data low byte */
#define TCS34725_RDATAL (0x16) /**< Red channel data low byte */
#define TCS34725_GDATAL (0x18) /**< Green channel data low byte */
#define TCS34725_BDATAL (0x1A) /**< Blue channel data low byte */

ColorSensor::ColorSensor() {
    addr = 0x32;
    green_threshold = 128;
}

void ColorSensor::write8(uint8_t reg, uint8_t value) {
//   uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), value};
//   i2c_dev->write(buffer, 2);
    uint8_t *source = &value;
    if(!I2C::writeRegisters(addr, reg, source, 1))
        std::cout << "Failed to Write 8 -- colorsensor.cpp" << std::endl; 
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 *  @param  reg
 *  @return value
 */
uint8_t ColorSensor::read8(uint8_t reg) {
//   uint8_t buffer[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
//   i2c_dev->write_then_read(buffer, 1, buffer, 1);
//   return buffer[0];
    uint8_t *dst;
    if(I2C::readRegisters(addr, reg, 1, dst))
        return *dst;
    else
        return 0x00;
}

/*!
 *  @brief  Reads a 16 bit values over I2C
 *  @param  reg
 *  @return value
 */
uint16_t ColorSensor::read16(uint8_t reg) {
//   uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), 0};
//   i2c_dev->write_then_read(buffer, 1, buffer, 2);
//   return (uint16_t(buffer[1]) << 8) | (uint16_t(buffer[0]) & 0xFF);
    uint8_t *dst;
    if(I2C::readRegisters(addr, reg, 2, dst))
        return *dst;
    else
        return 0x0000;
}

/*!
 *  @brief  Enables the device
 */
void ColorSensor::enable() {
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
//   delay(3);
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
  /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
//   delay((256 - _tcs34725IntegrationTime) * 12 / 5 + 1);
}

/*!
 *  @brief  Disables the device (putting it in lower power sleep mode)
 */
void ColorSensor::disable() {
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*!
 *  @brief  Reads the raw red, green, blue and clear channel values
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void ColorSensor::getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
//   if (!_tcs34725Initialised)
//     begin();

    *c = read16(TCS34725_CDATAL);
    *r = read16(TCS34725_RDATAL);
    *g = read16(TCS34725_GDATAL);
    *b = read16(TCS34725_BDATAL);

  /* Set a delay for the integration time */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
//   delay((256 - _tcs34725IntegrationTime) * 12 / 5 + 1);
}

/*!
 *  @brief  Reads the raw red, green, blue and clear channel values in
 *          one-shot mode (e.g., wakes from sleep, takes measurement, enters
 *          sleep)
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void ColorSensor::getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c) {
    enable();
    getRawData(r, g, b, c);
    disable();
}

/*!
 *  @brief  Read the RGB color detected by the sensor.
 *  @param  *r
 *          Red value normalized to 0-255
 *  @param  *g
 *          Green value normalized to 0-255
 *  @param  *b
 *          Blue value normalized to 0-255
 */
void ColorSensor::getRGB(float *r, float *g, float *b) {
    uint16_t red, green, blue, clear;
    getRawData(&red, &green, &blue, &clear);
    uint32_t sum = clear;

    // Avoid divide by zero errors ... if clear = 0 return black
    if (clear == 0) {
        *r = *g = *b = 0;
        return;
    }

    *r = (float)red / sum * 255.0;
    *g = (float)green / sum * 255.0;
    *b = (float)blue / sum * 255.0;
}

bool ColorSensor::is_start_light()
{
    float red, green, blue;
    getRGB(&red, &green, &blue);

    if (green > green_threshold)
        return true;
    
    return false;
}
