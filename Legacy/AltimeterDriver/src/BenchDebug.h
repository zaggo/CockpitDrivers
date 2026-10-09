#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "Altimeter.h"

class BenchDebug {
    public:
        BenchDebug();
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleAltimeterInput(String command);

        String inputBuffer;
                
        uint32_t fetchPressureRatio = 0L;
        float lastPressureRatio = -1.0f;

        float currentHeightInFeet = 0.0f;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        Altimeter* altimeter;
};
#endif
#endif // BENCHDEBUG_H