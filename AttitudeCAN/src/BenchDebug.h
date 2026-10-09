#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "AttitudeIndicator.h"

class BenchDebug {
    public:
        BenchDebug(AttitudeIndicator* attitude);

        void loop();
    private:
        void handleUserInput();
        bool handleCommand(String command);
        void printStatus();
        void hallMonitor();

        String inputBuffer;

        AttitudeIndicator* attitude;
};
#endif
#endif // BENCHDEBUG_H
