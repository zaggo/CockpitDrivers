#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "Rudder.h"

class BenchDebug {
    public:
        BenchDebug(Rudder* rudder);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleRudderInput(const char* command);
        bool handleCalibrationInput(const char* command);
        void printCalibration();
        void printState();
        void noiseProbe();

        // Fixed buffer instead of String: heap churn on an ATmega328 eats the
        // headroom the CAN path needs.
        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        // Live values are only interesting at reading speed on the bench; a
        // faster cadence floods the console and starves the sample loop.
        static const uint32_t kPrintIntervalMs = 100L;
        uint32_t lastPrintMs = 0L;

        Rudder* rudder;
};
#endif
#endif // BENCHDEBUG_H
