#include "Handbrake.h"
#include "DebugLog.h"

Handbrake::Handbrake() {
    pinMode(kHandbrakePin, INPUT);
}

Handbrake::~Handbrake() {
}

uint16_t Handbrake::getRawPosition() {
    return analogRead(kHandbrakePin);
}

uint8_t Handbrake::getHandbrakePosition() {
    uint16_t rawValue = getRawPosition();
    uint8_t position = map(rawValue, 0, 1023, 0, 255);
    return position;
}