#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "Starter.h"

class BenchDebug {
    public:
        BenchDebug(Starter* starter);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleStarterInput(String command);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        Starter* starter;
};
#endif
#endif // BENCHDEBUG_H
