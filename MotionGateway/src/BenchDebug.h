#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"

class BenchDebug {
    public:
        BenchDebug(CAN* canBus);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleBenchInput(String command);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        uint16_t actorDemand[6] = {0}; // Indexed by nodeId (1..3) and motor (M1/M2)
        CAN* canBus;
};

#endif // BENCHDEBUG_H