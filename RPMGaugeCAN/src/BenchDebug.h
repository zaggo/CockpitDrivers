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
        bool handleRPMGaugeInput(char* command);

        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        bool continuousTestActive = false;
        float continuousTestStartSeconds = 0;
        uint32_t continuousTestElapsedSeconds = 0;
        uint32_t lastSecondTick = 0;
        uint32_t nextMoveTime = 0;

        RPMGauge* rpmGauge;
        Odometer* odometer;
};
#endif
#endif // BENCHDEBUG_H
