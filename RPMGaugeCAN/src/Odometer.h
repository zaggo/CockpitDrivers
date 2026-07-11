#ifndef ODOMETER_H
#define ODOMETER_H
#include <Arduino.h>
#include "OLED0in91.h"

class Odometer
{ 
  public:
    Odometer();
    ~Odometer();

    void displayNumber(float digits[]);
    void secondsToDigits(float seconds, float* digits);
    void hoursToDigits(float hours, float* digits);
    inline void asyncTask() {
        oled->asyncTask();
    }

  private:
    OLED0in91* oled;

    uint8_t* currentDigits;
    // Tracks digits left mid-slide by the rollover-chain animation, so the next
    // displayNumber() call finalizes them even if their value didn't change.
    bool digitAnimating[6] = {};

    const uint16_t kLeftMargin = 0;
    const uint16_t kTopMargin = 2;
    const uint16_t kDigitWidth = 21;
};


#endif // ODOMETER_H