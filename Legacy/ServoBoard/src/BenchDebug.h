#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Configuration.h"
#include "Servos.h"

class BenchDebug {
    public:
        BenchDebug();
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        Servos* servos;
};

#endif // BENCHDEBUG_H