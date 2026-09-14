#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "WhiskeyCompass.h"

class BenchDebug {
    public:
        BenchDebug(WhiskeyCompass* compass);

        void loop();
    private:
        void handleUserInput();
        bool handleCommand(String command);
        void printStatus();

        String inputBuffer;

        WhiskeyCompass* compass;
};
#endif
#endif // BENCHDEBUG_H
