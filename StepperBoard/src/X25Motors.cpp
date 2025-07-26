#include "X25Motors.h"

X25Motors::X25Motors() {
    for (uint8_t i = 0; i < kX25MotorCount; i++) {
        x25Steppers[i] = new SwitecX25(kX25TotalSteps, kX25MotorPins[i][0], kX25MotorPins[i][1], kX25MotorPins[i][2], kX25MotorPins[i][3]);
        x25Steppers[i]->zero();       // Motor gegen Anschlag fahren
        x25Steppers[i]->setPosition(0); // Startposition setzen
    }
}

X25Motors::~X25Motors() {
    for (uint8_t i = 0; i < kX25MotorCount; i++) {
        delete x25Steppers[i];
    }
}

void X25Motors::updateAllX25Steppers() {
    // Motoren aktualisieren
    for (uint8_t i = 0; i < kX25MotorCount; i++) {
        x25Steppers[i]->update();
    }
}

void X25Motors::setPosition(uint8_t motorIndex, float relPos) {
    if (motorIndex >= kX25MotorCount) {
        return;
    }
    x25Steppers[motorIndex]->setPosition(relPos * kX25TotalSteps);
}