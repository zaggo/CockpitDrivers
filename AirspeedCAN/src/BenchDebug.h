#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "AirspeedIndicator.h"

class BenchDebug {
    public:
        BenchDebug(AirspeedIndicator* airspeedIndicator);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleAirspeedInput(char* command);
        void printCalibration();

        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        static const uint16_t kIASDeliveryIntervalMs = 20; // 50Hz, matches real CAN cadence

        bool continuousTestActive = false;

        // Random IAS target is not applied in one jump - it's interpolated and
        // delivered via airspeedIndicator->moveNeedle() at kIASDeliveryIntervalMs,
        // just like real 50Hz CAN telemetry would.
        float iasMoveFrom = 0;
        float iasMoveTo = 0;
        uint32_t iasMoveStartMillis = 0;
        uint32_t iasMoveDurationMillis = 0;
        uint32_t lastIASDeliveryMillis = 0;

        AirspeedIndicator* airspeedIndicator;
};
#endif
#endif // BENCHDEBUG_H
