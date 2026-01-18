#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "CAN.h"

class BenchDebug {
    public:
        BenchDebug(CAN* canBus);
        ~BenchDebug();

        void loop();
    private:
        void handleUserInput();
        bool handleAltimeterInput(String command);

        void sendFuelLevel();
        void sendCockpitLightLevel();

        String inputBuffer;

        float leftTankLevelKg = 0.;
        float rightTankLevelKg = 0.;

        uint8_t cockpitLightLevel = 0;
                
        CAN* canBus;
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;
};
#endif
#endif // BENCHDEBUG_H