#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "VerticalSpeedIndicator.h"

class BenchDebug {
    public:
        BenchDebug(VerticalSpeedIndicator* verticalSpeedIndicator);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleVerticalSpeedInput(char* command);
        void printCalibration();

        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        static const uint16_t kVSIDeliveryIntervalMs = 20; // 50Hz, matches real CAN cadence

        bool continuousTestActive = false;

        // Random fpm target is not applied in one jump - it's interpolated and
        // delivered via verticalSpeedIndicator->moveNeedle() at
        // kVSIDeliveryIntervalMs, just like real 50Hz CAN telemetry would.
        float vsiMoveFrom = 0;
        float vsiMoveTo = 0;
        uint32_t vsiMoveStartMillis = 0;
        uint32_t vsiMoveDurationMillis = 0;
        uint32_t lastVSIDeliveryMillis = 0;

        VerticalSpeedIndicator* verticalSpeedIndicator;
};
#endif
#endif // BENCHDEBUG_H
