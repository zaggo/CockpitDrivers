#include "X25Motors.h"

void initX25Steppers() {
    for (uint8_t i = 0; i < kX25MotorCount; i++) {
        x25Steppers[i] = new SwitecX25(kX25TotalSteps, kX25MotorPins[i][0], kX25MotorPins[i][1], kX25MotorPins[i][2], kX25MotorPins[i][3]);
        x25Steppers[i]->zero();       // Motor gegen Anschlag fahren
        x25Steppers[i]->setPosition(0); // Startposition setzen
    }
}

void updateAllX25Steppers() {
    // Motoren aktualisieren
    for (uint8_t i = 0; i < kX25MotorCount; i++) {
        x25Steppers[i]->update();
    }
}