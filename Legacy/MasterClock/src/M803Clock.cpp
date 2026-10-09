#include "M803Clock.h"

M803Clock::M803Clock()
{
  pinMode(kContrastPin, OUTPUT);
  analogWrite(kContrastPin, 40);

  // lcd = new LiquidCrystal(4, 6, 5, 7, 8, 9, 10);
  lcd = new LiquidCrystal_I2C(0x27, 16, 2);
  // lcd->begin(16, 2);
  lcd->init(); // initialize the lcd
  lcd->begin(16, 2);
  lcd->clear();

  // Print a message to the LCD.
  lcd->backlight();

  // lcd->createChar(0, kUOff);
  // lcd->createChar(1, kLOff);
  // lcd->createChar(2, kFOff);
  // lcd->createChar(3, kEOff);
  // lcd->createChar(4, kT);
  // lcd->createChar(5, kOn);

  delay(1000);
  lcd->noBacklight();
  delay(500);
  lcd->backlight();
  delay(1000);
  lcd->noBacklight();
  delay(500);
  lcd->backlight();
  delay(1000);
  lcd->backlight();

  lcd->cursor();

  // lcd->write((uint8_t)0);
  // lcd->write((uint8_t)4);
  // lcd->write((uint8_t)5);
  // lcd->write((uint8_t)1);
  // lcd->write((uint8_t)4);

  // lcd->print("  12.1E");

  // lcd->setCursor(0, 1);
  // lcd->write((uint8_t)2);
  // lcd->write((uint8_t)4);
  // lcd->write((uint8_t)32);
  // lcd->write((uint8_t)3);
  // lcd->write((uint8_t)4);

  lcd->print("  12:50");
}