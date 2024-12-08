#ifndef OLED0IN91_H
#define OLED0IN91_H

#include <Wire.h>
#include <avr/pgmspace.h>

typedef struct {
    uint8_t *image;
    uint16_t width;
    uint16_t height;
    uint16_t widthMemory;
    uint16_t heightMemory;
    uint16_t color;
    uint16_t widthByte;
    uint16_t heightByte;
} Canvas;

class OLED0in91
{
    public:
        OLED0in91();
        ~OLED0in91();

        void displayCanvas();

        void fillRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, bool white);
        void drawVerticalLine(uint16_t xStart, uint16_t yStart, uint16_t length, bool white);
        void drawDigit(int16_t x, int16_t y, const uint8_t digit, bool white, bool onBlack);

    private:
        void initOLEDRegisters();

        inline void writeOLEDRegister(uint8_t reg)
        {
            writeByteToI2C(reg, IIC_CMD);
        }

        inline void writeOLEDData(uint8_t data) 
        {
            writeByteToI2C(data, IIC_RAM);
        }

        inline void writeByteToI2C(uint8_t value, uint8_t cmd)
        {
            uint8_t addr = 0x3c;
            Wire.beginTransmission(addr);
            Wire.write(cmd);
            Wire.write(value);
            Wire.endTransmission();
        }

        void setPixel(int16_t x, int16_t y, bool white);

        // iVars
        Canvas canvas;

        // Constants
        const uint8_t IIC_CMD = 0x00;
        const uint8_t IIC_RAM = 0x40;
        const uint8_t OLED_0in91_WIDTH = 128; // 0.91 inch OLED width
        const uint8_t OLED_0in91_HEIGHT = 32; // 0.91 inch OLED height
};

#endif // OLED0IN91_H