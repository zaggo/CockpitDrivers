#ifndef HANDBRAKE_H
#define HANDBRAKE_H
#include <Arduino.h>
#include "Configuration.h"

class Handbrake
{
public:
    Handbrake();
    ~Handbrake();

    uint8_t getHandbrakePosition();
    uint16_t getRawPosition();
};

#endif