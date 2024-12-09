#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "GyroDrive.h"
#include "Configuration.h"
#include "X25Motors.h"

enum BenchMode {
    kGyroDrive = 1 << 0,
    kX25Motors = 1 << 1,

    kAll = kGyroDrive | kX25Motors
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

        GyroDrive* gyroDrive;
        X25Motors* x25Motors;
};

#endif // BENCHDEBUG_H