#ifndef TACHOMETER_H
#define TACHOMETER_H
#include <Arduino.h>
#include "OLED0in91.h"

class Tachometer {
  public:
    Tachometer();
    ~Tachometer();

    void displayNumber(float digits[]);
    void secondsToDigits(float seconds, float* digits);
    inline void asyncTask() {
        oled->asyncTask();
    }

  private:
    OLED0in91* oled;

    uint8_t* currentDigits;
    int16_t currentYShift = 0;

    const uint16_t kLeftMargin = 0;
    const uint16_t kTopMargin = 2;
    const uint16_t kDigitWidth = 21;
};


#endif // TACHOMETER_H