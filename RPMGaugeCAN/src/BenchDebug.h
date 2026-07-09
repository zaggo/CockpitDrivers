#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "RPMGauge.h"
#include "Odometer.h"

class BenchDebug {
    public:
        BenchDebug(RPMGauge* rpmGauge, Odometer* odometer);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleRPMGaugeInput(String command);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        RPMGauge* rpmGauge;
        Odometer* odometer;
};
#endif
#endif // BENCHDEBUG_H
