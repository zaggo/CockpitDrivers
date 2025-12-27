#ifndef SERVOS_H
#define SERVOS_H
#include "Servos.h"
#include "Configuration.h"
#include <Servo.h>

class Servos {
public:
    Servos();
    ~Servos();
    
    void setPosition(int servoId, float position);
    void setPositions(int count, float* positions);
    
private:
    Servo servos[kServoCount];

    int mapServoValue(float relativeValue) {
        // Map the relative value to a range suitable for the servo
        // Assuming relativeValue is between 0.0 and 1.0
        return static_cast<int>(relativeValue * 180); // Servo.write expects degrees
    }
};

#endif // SERVOS_H