#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Tachometer.h"

class BenchDebug {
    public:
        BenchDebug();
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
};

#endif // BENCHDEBUG_H