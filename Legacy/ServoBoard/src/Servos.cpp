   #include "Servos.h"
#include "Configuration.h"

#include <Servo.h>

Servos::Servos() {
    for (int i = 0; i < kServoCount; i++) {
        servos[i].attach(kServoPins[i]);
    }
}

Servos::~Servos() {
    for (int i = 0; i < kServoCount; i++) {
        servos[i].detach();
    }
}

void Servos::setPosition(int servoId, float position) {
    if (servoId >= 0 && servoId < kServoCount) {
        servos[servoId].write(mapServoValue(position));
    }
}

void Servos::setPositions(int count, float* positions) {
    for (int i = 0; i < count && i < kServoCount; i++) {
        servos[i].write(mapServoValue(positions[i]));
    }
}