#ifndef BENCHDEBUG_H
#define BENCHDEBUG_H
#include "Configuration.h"

#if BENCHDEBUG
#include <Arduino.h>
#include "CAN.h"
#include <CanMessageId.h>
#include <CanNodeId.h>

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
        void sendRpm();
        void sendOdometer();

        void startRudderWatch();
        void stopRudderWatch();
        void handleRudderWatch();

        String inputBuffer;

        float leftTankLevelKg = 0.;
        float rightTankLevelKg = 0.;

        uint8_t cockpitLightLevel = 0;

        uint16_t rpmValue = 0;
        float odometerHours = 0.;

        // Rudder watch: prints incoming 0x303 frames until any key is pressed.
        bool rudderWatchActive = false;
        bool rudderWatchPrinted = false; // false = print the next sample unconditionally
        RudderToDcuMessage lastRudderPrinted = {0, 0, 0};


        CAN* canBus;
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;
};
#endif
#endif // BENCHDEBUG_H