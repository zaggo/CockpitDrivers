#include "Tachometer.h"

Tachometer::Tachometer()
{
    oled = new OLED0in91();
    currentDigits = new uint8_t[6];

    // Initialize currentDigits with 0xff
    for (uint8_t i = 0; i < 6; i++)
    {
        currentDigits[i] = 0xff;
    }

    float zero[] = {0., 0., 0., 0., 0., 0.};
    displayNumber(zero);
}

Tachometer::~Tachometer()
{
    delete oled;
    delete[] currentDigits;
}

// Transforms a 6 digit number into a 6 element array
void Tachometer::secondsToDigits(float seconds, float *digits)
{
    float decimalHours = seconds / 3600.0;
    uint32_t number = static_cast<uint32_t>(decimalHours * 100);
    for (uint8_t i = 0; i < 6; i++)
    {
        digits[5 - i] = static_cast<float>(number % 10);
        number /= 10;
    }
    digits[5] += static_cast<float>((decimalHours * 100) - static_cast<uint32_t>(decimalHours * 100));
}

// Displays a 6 digit number on the OLED display
void Tachometer::displayNumber(float digits[])
{
    bool somethingChanged = false;

    float fraction = digits[5] - static_cast<uint8_t>(digits[5]);
    int16_t yShift = 0;

    if(fraction >= 0.9) {
        float animation = (fraction - 0.9) * 10.0;
        yShift = static_cast<int16_t>(animation * 32);
    }

    for (uint8_t i = 0; i < 6; i++)
    {
        uint8_t digit = static_cast<uint8_t>(digits[i]);
        bool isAnimating = false;
        if (i < 5) {
            isAnimating = (static_cast<uint8_t>(digits[i+1]) == 9 && yShift != currentYShift);
        } else {
            isAnimating = (yShift != currentYShift);
        }
        if (currentDigits[i] != digit || isAnimating)
        {
            uint16_t charX = kLeftMargin + i * kDigitWidth;

            bool isWhiteOnBlack = i < 4;
            if (!isWhiteOnBlack)
            {
                oled->fillRectangle(charX, 0, 19, 32, true);
            }
            else
            {
                oled->fillRectangle(charX, 0, kDigitWidth, 32, false);
            }

            if (isAnimating)
            {
                uint8_t nextDigit = (digit + 1) % 10;

                oled->drawDigit(charX + (isWhiteOnBlack ? 0 : 1), kTopMargin - yShift, digit, isWhiteOnBlack, isWhiteOnBlack);
                oled->drawDigit(charX + (isWhiteOnBlack ? 0 : 1), kTopMargin + 32 - yShift, nextDigit, isWhiteOnBlack, isWhiteOnBlack);
            }
            else
            {
                oled->drawDigit(charX + (isWhiteOnBlack ? 0 : 1), kTopMargin, digit, isWhiteOnBlack, isWhiteOnBlack);
            }
            currentDigits[i] = digit;
            if (i == 5)
            {
                currentYShift = yShift;
            }
            somethingChanged = true;
        }
    }

    if (somethingChanged)
    {
        oled->displayCanvas();    
        //Serial.println("Displaying canvas");
    }
}
