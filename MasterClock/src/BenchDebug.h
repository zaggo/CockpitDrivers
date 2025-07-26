#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Tachometer.h"
#include "M803Clock.h"

enum BenchMode {
    kTachometer = 1 << 0,
    kM803Clock = 1 << 1,

    kAll = 0xff
};

class BenchDebug {
    public:
        BenchDebug(BenchMode mode);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        uint32_t startTime;
        uint32_t lastTime = 0L;
        float* digits;
        uint16_t lastSeconds = 0;

        Tachometer* tachometer;
        M803Clock* m803Clock;
};

#endif // BENCHDEBUG_H