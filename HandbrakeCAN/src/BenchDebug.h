#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "Handbrake.h"

class BenchDebug {
    public:
        BenchDebug(Handbrake* handbrake);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleHandbrakeInput(String command);

        String inputBuffer;
                
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        uint8_t position = 0;

        Handbrake* handbrake;
};
#endif
#endif // BENCHDEBUG_H