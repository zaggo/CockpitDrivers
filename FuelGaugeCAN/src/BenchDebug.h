#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "FuelGauge.h"

class BenchDebug {
    public:
        BenchDebug(FuelGauge* fuelGauge);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleAltimeterInput(String command);

        String inputBuffer;
                
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        FuelGauge* fuelGauge;
};
#endif
#endif // BENCHDEBUG_H