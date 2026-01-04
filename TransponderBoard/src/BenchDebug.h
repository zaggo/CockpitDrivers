#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Configuration.h"
#include "Transponder.h"

class BenchDebug {
    public:
        BenchDebug();
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleBenchInput(String command);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        Transponder* transponder;
};

#endif // BENCHDEBUG_H