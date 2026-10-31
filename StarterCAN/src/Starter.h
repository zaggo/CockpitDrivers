#ifndef STARTER_H
#define STARTER_H
#include <Arduino.h>
#include "Configuration.h"

struct StarterPositionUpdate {
    bool changed;
    uint8_t position;
};

class Starter
{
public:
    Starter();
    ~Starter();

    // Currently accepted switch position, 0..kSwitchPositionCount-1
    // (0 = OFF, 1 = R, 2 = L, 3 = BOTH, 4 = START).
    uint8_t getPosition() const;

    // Debounced change detection. Call every loop; reports changed = true once
    // a new detent has read stable for kSwitchDebounceMs. The very first
    // reading after power-up also counts as a change, so the current position
    // is always reported at least once.
    StarterPositionUpdate getPositionUpdate();

    // Starter LED: on at kStarterLedBrightness, off at 0.
    void setLed(bool on);
    bool getLed() const;

private:
    // Index of the currently grounded pin, or kSwitchPositionCount when the
    // switch sits between detents (no pin grounded).
    uint8_t readRawPosition() const;

    uint8_t _position = 0;
    bool _hasPosition = false;
    bool _ledOn = false;

    // Debounce state: the candidate detent and since when it reads stable.
    uint8_t _candidate = kSwitchPositionCount;
    unsigned long _candidateSinceMs = 0;
};

#endif // STARTER_H
