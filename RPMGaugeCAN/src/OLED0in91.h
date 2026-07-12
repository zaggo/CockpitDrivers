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

        void asyncDisplayCanvas();
        void asyncTask();

        void fillRectangle(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd, bool white);
        void drawVerticalLine(uint16_t xStart, uint16_t yStart, uint16_t length, bool white);
        void drawDigit(int16_t x, int16_t y, const uint8_t digit, bool white, bool onBlack);

    private:
        void initOLEDRegisters();

        inline void writeOLEDRegister(uint8_t reg)
        {
            writeByteToI2C(reg, IIC_CMD);
        }

        inline void writeByteToI2C(uint8_t value, uint8_t cmd)
        {
            uint8_t addr = 0x3c;
            Wire.beginTransmission(addr);
            Wire.write(cmd);
            Wire.write(value);
            Wire.endTransmission();
        }

        // Gathers up to kI2CChunkSize columns of one strided image line (stride 4)
        // and sends them as a single I2C transmission, instead of one start/stop
        // per byte.
        void sendChunk(const uint8_t* imageBuffer, uint8_t line, uint8_t startColumn, uint8_t len);

        // Sends one full strided image page (128 columns) as consecutive chunks.
        void sendPage(const uint8_t* imageBuffer, uint8_t line);

        void setPixel(int16_t x, int16_t y, bool white);

        // iVars
        Canvas canvas;
        uint8_t *asyncImageBuffer;
        uint8_t asyncLine = OLED_0in91_HEIGHT/8;
        uint8_t asyncColumn = 0;

        // Constants
        const uint8_t IIC_CMD = 0x00;
        const uint8_t IIC_RAM = 0x40;
        const uint8_t OLED_0in91_WIDTH = 128; // 0.91 inch OLED width
        const uint8_t OLED_0in91_HEIGHT = 32; // 0.91 inch OLED height
        const uint8_t kI2CChunkSize = 31; // Wire's TWI buffer is 32 bytes; 1 goes to the control byte
};

#endif // OLED0IN91_H