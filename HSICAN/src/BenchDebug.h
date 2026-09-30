#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "HSI.h"

class BenchDebug {
    public:
        BenchDebug(HSI* hsi);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleHSIInput(String command);
        void applyAll();
        void resetHeadingMirror();
        void measureBacklash(HSIAxis axis);
        static uint16_t wrapDeg100(float degrees);
        static HSIAxis axisFromArg(const String& arg);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        // Mirrors what 0x104 / 0x105 would carry, so one command updates its
        // own field and the whole picture is reapplied.
        uint16_t heading100 = 0;
        uint16_t obs100 = 0;
        uint16_t bug100 = 0;
        uint8_t  flags = 0;
        int16_t  cdiDots1000 = 0;
        int16_t  gsDots1000 = 0;

        void monitorHalls();

        bool hallMonitor = false;
        uint8_t lastHallBits = 0;
        uint16_t hallFlips = 0;
        uint32_t lastHallPrintMs = 0;

        uint32_t pendingBacklash = 0;
        HSIAxis pendingBacklashAxis = obs;

        HSI* hsi;
};
#endif
#endif // BENCHDEBUG_H