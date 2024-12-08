#include "OLED0in91.h"
#include <Arduino.h>
#include "DebugLog.h"
#include "DigitsFont.h"

OLED0in91::OLED0in91()
{
    Wire.setClock(1000000);
    Wire.begin();

    // Set the initialization register
    initOLEDRegisters();
    delay(200);

    // Turn on the OLED display
    writeOLEDRegister(0xaf);

    // The canvas is rotated 90 degrees in memory
    canvas.widthMemory = OLED_0in91_HEIGHT;
    canvas.heightMemory = OLED_0in91_WIDTH;
    canvas.widthByte = (OLED_0in91_HEIGHT % 8 == 0) ? (OLED_0in91_HEIGHT / 8) : (OLED_0in91_HEIGHT / 8 + 1);
    canvas.heightByte = OLED_0in91_WIDTH;

    canvas.width = OLED_0in91_WIDTH;
    canvas.height = OLED_0in91_HEIGHT;

    canvas.image = new uint8_t[canvas.widthByte * canvas.heightByte]();
    if (canvas.image == NULL)
    {
        DEBUGLOG_PRINTLN("Failed to apply for black memory...");
    }

    displayCanvas();
}

OLED0in91::~OLED0in91()
{
    Wire.end();
    delete[] canvas.image;
}

void OLED0in91::displayCanvas()
{
    for (uint8_t line = 0; line < OLED_0in91_HEIGHT / 8; line++)
    {
        writeOLEDRegister(0xb0 + line);
        writeOLEDRegister(0x00);
        writeOLEDRegister(0x10);
        for (uint8_t column = 0; column < OLED_0in91_WIDTH; column++)
        {
            writeOLEDData(canvas.image[(3 - line) + column * 4]);
        }
    }
}

// Service
void OLED0in91::initOLEDRegisters()
{
    writeOLEDRegister(0xAE);

    writeOLEDRegister(0x40); //---set low column address
    writeOLEDRegister(0xB0); //---set high column address

    writeOLEDRegister(0xC8); //-not offset

    writeOLEDRegister(0x81);
    writeOLEDRegister(0xff);

    writeOLEDRegister(0xa1);

    writeOLEDRegister(0xa6);

    writeOLEDRegister(0xa8);
    writeOLEDRegister(0x1f);

    writeOLEDRegister(0xd3);
    writeOLEDRegister(0x00);

    writeOLEDRegister(0xd5);
    writeOLEDRegister(0xf0);

    writeOLEDRegister(0xd9);
    writeOLEDRegister(0x22);

    writeOLEDRegister(0xda);
    writeOLEDRegister(0x02);

    writeOLEDRegister(0xdb);
    writeOLEDRegister(0x49);

    writeOLEDRegister(0x8d);
    writeOLEDRegister(0x14);
}

// API
void OLED0in91::drawDigit(int16_t x, int16_t y, const uint8_t digit, bool white, bool onBlack)
{
    uint32_t charOffset = digit * digitsFont.height * (digitsFont.width / 8 + (digitsFont.width % 8 ? 1 : 0));
    const unsigned char *ptr = &digitsFont.table[charOffset];

    for (int16_t line = 0; line < digitsFont.height; line++)
    {
        for (int16_t column = 0; column < digitsFont.width; column++)
        {
            // To determine whether the font background color and screen background color is consistent
            if (onBlack)
            { // this process is to speed up the scan
                if (pgm_read_byte(ptr) & (0x80 >> (column % 8)))
                    setPixel(x + column, y + line, white);
            }
            else
            {
                if (pgm_read_byte(ptr) & (0x80 >> (column % 8)))
                {
                    setPixel(x + column, y + line, white);
                }
                else
                {
                    setPixel(x + column, y + line, !white);
                }
            }
            // One pixel is 8 bits
            if (column % 8 == 7)
            {
                ptr++;
            }
        } /* Write a line */
        if (digitsFont.width % 8 != 0)
        {
            ptr++;
        }
    } /* Write all */
}

void OLED0in91::setPixel(int16_t x, int16_t y, bool white)
{
    if (x < 0 || y < 0 || x >= canvas.width || y >= canvas.height)
    {
        return;
    }
    uint16_t cX = canvas.widthMemory - y - 1;
    uint16_t cY = x;

    uint32_t addr = cX / 8 + cY * canvas.widthByte;
    uint8_t rData = canvas.image[addr];
    if (white)
    {
        canvas.image[addr] = rData | (0x80 >> (cX % 8));
    }
    else
    {
        canvas.image[addr] = rData & ~(0x80 >> (cX % 8));
    }
}

void OLED0in91::fillRectangle(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bool white)
{
    uint16_t xEnd = x + width;
    if (x > canvas.width || y > canvas.height ||
        xEnd > canvas.width || y + height > canvas.height)
    {
        DEBUGLOG_PRINTLN("Out of bounds");
        return;
    }

    for(uint16_t i = x; i <= xEnd; i++)
    {
        drawVerticalLine(i, y, height, white);
    }
}

void OLED0in91::drawVerticalLine(uint16_t x, uint16_t y, uint16_t length, bool white)
{
    uint16_t yEnd = y + length;
    if (x > canvas.width || y > canvas.height || yEnd > canvas.height)
    {
        DEBUGLOG_PRINTLN("Out of bounds");
        return;
    }

    for(uint16_t i = y; i <= yEnd; i++)
    {
        setPixel(x, i, white);
    }
}