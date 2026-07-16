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

        static const uint16_t kOdometerDeliveryIntervalMs = 100; // 10Hz, matches real DCU cadence (10x sim-time speed)
        static const uint16_t kRPMDeliveryIntervalMs = 20; // 50Hz, matches real CAN cadence

        bool continuousTestActive = false;
        float continuousTestStartSeconds = 0;
        uint32_t continuousTestElapsedSeconds = 0;
        uint32_t lastSecondTick = 0;

        // Random RPM target is not applied in one jump - it's interpolated and
        // delivered via rpmGauge->moveNeedle() at kRPMDeliveryIntervalMs, just
        // like real 50Hz CAN telemetry would.
        float rpmMoveFrom = 0;
        float rpmMoveTo = 0;
        uint32_t rpmMoveStartMillis = 0;
        uint32_t rpmMoveDurationMillis = 0;
        uint32_t lastRPMDeliveryMillis = 0;

        RPMGauge* rpmGauge;
        Odometer* odometer;
};
#endif
#endif // BENCHDEBUG_H
