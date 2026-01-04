#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "GyroDrive.h"
#include "Configuration.h"
#include "X25Motors.h"
#include "Transponder.h"

class BenchDebug {
    public:
        BenchDebug(BenchMode mode);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleBenchInput(String command);

        String inputBuffer;
        int16_t pitchTargetDegrees = 0;
        int16_t rollTargetDegrees = 0;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        GyroDrive* gyroDrive;
        X25Motors* x25Motors;
        Transponder* transponder;
};

#endif // BENCHDEBUG_H