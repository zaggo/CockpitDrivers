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
        void printState();

        // Fixed buffer instead of String: heap churn on an ATmega328 eats the
        // headroom the CAN path needs.
        static const uint8_t kInputBufferSize = 32;
        char inputBuffer[kInputBufferSize];
        uint8_t inputLength = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        uint32_t lastPrintMs = 0L;

        Rudder* rudder;
};
#endif
#endif // BENCHDEBUG_H
