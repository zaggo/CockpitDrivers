#include "Starter.h"
#include "DebugLog.h"

Starter::Starter() {
    for (uint8_t i = 0; i < kSwitchPositionCount; i++) {
        pinMode(kSwitchPins[i], INPUT_PULLUP);
    }
    pinMode(kStarterLedPin, OUTPUT);
    setLed(false);
}

Starter::~Starter() {
}

uint8_t Starter::readRawPosition() const {
    for (uint8_t i = 0; i < kSwitchPositionCount; i++) {
        if (digitalRead(kSwitchPins[i]) == LOW) {
            return i;
        }
    }
    return kSwitchPositionCount; // between detents
}

uint8_t Starter::getPosition() const {
    return _position;
}

StarterPositionUpdate Starter::getPositionUpdate() {
    StarterPositionUpdate update;
    update.changed = false;
    update.position = _position;

    const uint8_t raw = readRawPosition();
    const unsigned long now = millis();

    if (raw != _candidate) {
        _candidate = raw;
        _candidateSinceMs = now;
        return update;
    }

    // Between detents nothing is grounded: keep the last position.
    if (_candidate == kSwitchPositionCount) {
        return update;
    }

    if ((unsigned long)(now - _candidateSinceMs) < kSwitchDebounceMs) {
        return update;
    }

    if (!_hasPosition || _candidate != _position) {
        _hasPosition = true;
        _position = _candidate;
        update.changed = true;
        update.position = _position;
    }

    return update;
}

void Starter::setLed(bool on) {
    _ledOn = on;
    analogWrite(kStarterLedPin, on ? kStarterLedBrightness : 0);
}

bool Starter::getLed() const {
    return _ledOn;
}
