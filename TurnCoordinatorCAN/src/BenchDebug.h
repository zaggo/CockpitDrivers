#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "TurnCoordinator.h"

class BenchDebug {
    public:
        BenchDebug(TurnCoordinator* turnCoordinator);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleTurnCoordinatorInput(char* command);
        void printCalibration();
        void printTable(const __FlashStringHelper* name, const CalibrationTable& table);

        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        static const uint16_t kDeliveryIntervalMs = 20; // 50Hz, matches real CAN cadence

        bool continuousTestActive = false;

        // Random targets per needle are not applied in one jump - they are
        // interpolated and delivered via turnCoordinator->moveNeedle() at
        // kDeliveryIntervalMs, just like real 50Hz CAN telemetry would.
        struct NeedleMove {
            float from = 0;
            float to = 0;
            uint32_t startMillis = 0;
            uint32_t durationMillis = 0;
        };
        NeedleMove moves[needleCount];
        uint32_t lastDeliveryMillis = 0;

        TurnCoordinator* turnCoordinator;
};
#endif
#endif // BENCHDEBUG_H
