#ifndef LCD_H
#define LCD_H
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_PCF8574.h>

class LCD {
    public:
        LCD();
        bool begin();
        void printFirstLine(const String& message);
        void printSecondLine(const String& message);
        void setBacklightPWM(uint8_t pwm);
    private:
        LiquidCrystal_PCF8574 lcd;
};

#endif // LCD_H