#include "Odometer.h"

Odometer::Odometer()
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

Odometer::~Odometer()
{
    delete oled;
    delete[] currentDigits;
}

// Transforms a 6 digit number into a 6 element array
void Odometer::secondsToDigits(float seconds, float *digits)
{
    hoursToDigits(seconds / 3600.0, digits);
}

// Transforms a decimal hours value into a 6 element digit array
void Odometer::hoursToDigits(float hours, float *digits)
{
    uint32_t number = static_cast<uint32_t>(hours * 100);
    for (uint8_t i = 0; i < 6; i++)
    {
        digits[5 - i] = static_cast<float>(number % 10);
        number /= 10;
    }
    digits[5] += static_cast<float>((hours * 100) - static_cast<uint32_t>(hours * 100));
}

// Displays a 6 digit number on the OLED display
void Odometer::displayNumber(float digits[])
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
        bool chainEligible = true;
        for(uint8_t j = i + 1; j < 6; j++) {
            if (static_cast<uint8_t>(digits[j]) != 9) {
                chainEligible = false;
                break;
            }
        }
        // Depend only on whether we're mid-slide right now, not on whether yShift
        // moved since the last frame - yShift is quantized, so it can plateau across
        // several consecutive frames while still mid-animation.
        bool isAnimating = chainEligible && yShift > 0;
        if (currentDigits[i] != digit || isAnimating || digitAnimating[i])
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
            digitAnimating[i] = isAnimating;
            somethingChanged = true;
        }
    }

    if (somethingChanged)
    {
        oled->asyncDisplayCanvas();    
    }
}
