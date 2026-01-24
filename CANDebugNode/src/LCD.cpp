#include "LCD.h"

LCD::LCD() : lcd(0x27) // Default I2C address for PCF8574
{ 
}

bool LCD::begin()
{
    // See http://playground.arduino.cc/Main/I2cScanner how to test for a I2C device.
    Wire.begin();
    Wire.beginTransmission(0x27);
    uint8_t error = Wire.endTransmission();

    if (error != 0)
    {
        return false; // LCD not found
    }

    lcd.begin(16, 2);
    lcd.setBacklight(255);
    return true;
}

void LCD::printFirstLine(const String& message)
{
    lcd.setCursor(0, 0);
    lcd.print(message);
    if (message.length() < 16) {
        // Clear remaining characters on the line
        for (int i = message.length(); i < 16; i++) {
            lcd.print(" ");
        }
    }
}

void LCD::printSecondLine(const String& message)
{
    lcd.setCursor(0, 1);
    lcd.print(message);
    if (message.length() < 16) {
        // Clear remaining characters on the line
        for (int i = message.length(); i < 16; i++) {
            lcd.print(" ");
        }
    }
}

void LCD::setBacklightPWM(uint8_t pwm)
{
    lcd.setBacklight(pwm);
}