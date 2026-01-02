#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "HSI.h"

class BenchDebug {
    public:
        BenchDebug();
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleHSIInput(String command);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        double combiCompassDegree = 0.0;
        double combiHdgDegree = 0.0;
        double combiCdiDegree = 0.0;

        HSI* hsi;
};
#endif
#endif // BENCHDEBUG_H