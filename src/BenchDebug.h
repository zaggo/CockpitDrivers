#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Tachometer.h"
#include "GyroDrive.h"
#include "Configuration.h"
#include "X25Motors.h"

enum BenchMode {
    kGyroDrive = 1 << 0,
    kX25Motors = 1 << 1,
    kTachometer = 1 << 2,

    kAll = kGyroDrive | kX25Motors | kTachometer
};

class BenchDebug {
    public:
        BenchDebug(BenchMode mode);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        
        String inputBuffer;
        int16_t pitchTargetDegrees = 0;
        int16_t rollTargetDegrees = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        uint32_t startTime;
        uint32_t lastTime = 0L;
        float* digits;
        uint16_t lastSeconds = 0;

        GyroDrive* gyroDrive;
        X25Motors* x25Motors;
        Tachometer* tachometer;
};

#endif // BENCHDEBUG_H