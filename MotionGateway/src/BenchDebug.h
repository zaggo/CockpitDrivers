#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"

class BenchDebug {
    public:
        BenchDebug(CAN* canBus);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleBenchInput(String command);
        bool handleTestCommand(const String& command);
        bool handleStreamCommand(const String& command);
        void tickStreamGenerator();
        void sendPairDemand(uint8_t nodeId, uint16_t demand);

        String inputBuffer;

        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;

        uint16_t actorDemand[6] = {0}; // Indexed by nodeId (1..3) and motor (M1/M2)

        // gs stream generator: absolute-deadline 0x110 demand stream, so stage-1/2
        // experiments are reproducible without X-Plane.
        bool streamActive = false;
        uint8_t streamNodeId = 1;
        uint8_t streamWave = 0; // 0 = triangle, 1 = sine
        uint8_t streamAmpPct = 30;
        uint32_t streamPeriodUs = 16667;
        uint32_t streamEndMs = 0;
        uint32_t streamStartMs = 0;
        uint32_t streamNextTickUs = 0;
        uint32_t streamFramesSent = 0;
        uint32_t streamMissedTicks = 0;

        CAN* canBus;
};

#endif // BENCHDEBUG_H