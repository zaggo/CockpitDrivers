#ifndef HANDBRAKE_H
#define HANDBRAKE_H
#include <Arduino.h>
#include "Configuration.h"

struct HandbrakeConfig {
    uint32_t magic;           // identifies a valid config record
    uint16_t version;         // struct version for future extensions
    uint16_t minRawPosition;
    uint16_t maxRawPosition;
};

struct HandbrakePositionUpdate {
    bool changed;
    uint8_t position;
};

class Handbrake
{
public:
    Handbrake();
    ~Handbrake();

    uint8_t getHandbrakePosition();
    uint16_t getRawPosition();
    HandbrakePositionUpdate getPositionUpdate();

    void calibrateMin();
    void calibrateMax();

private:
    HandbrakeConfig _config;
    bool _hasLastReportedPosition;
    uint8_t _lastReportedPosition;
    void loadConfig();
    void saveConfig();
    uint16_t sampleAverage(uint8_t count);
};

#endif