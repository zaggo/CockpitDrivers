#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <avr/pgmspace.h>

enum ServoId {
    altAmp = 0,
    fuelPressure,
    trim,
    servor4,
    servor5,
    servor6,
    servor7,
    servor8,
    
    kServoCount
};

const int kServoPins[kServoCount] = { 2, 3, 4, 5, 6, 7, 8, 9 };

#endif // CONFIGURATION_H
