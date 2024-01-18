#include <stdint.h>
#include <memory>

#include "./I2C.h"

#ifndef COLORSENSORH
#define COLORSENSORH

class ColorSensor{
    public:
        ColorSensor();
        bool is_start_light();
    private:
        uint8_t addr;
        uint8_t read8(uint8_t reg);
        uint16_t read16(uint8_t reg);
        void write8(uint8_t reg, uint8_t value);
        void enable();
        void disable();
        void getRawData(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
        void getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b, uint16_t *c);
        void getRGB(float *r, float *g, float *b);
        
        uint16_t green_threshold;
};

#endif