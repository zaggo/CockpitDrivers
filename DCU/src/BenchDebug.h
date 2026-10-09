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
        void sendAirspeed();
        void sendAltimeterVsi();
        void sendCompass();
        void sendTurnCoordinator();

        void startRudderWatch();
        void stopRudderWatch();
        void handleRudderWatch();

        void startBaroWatch();
        void stopBaroWatch();
        void handleBaroWatch();

        // HSI bench state. One command updates its own field and reships the
        // whole frame, the way al/vs already share the altimeterVsi frame.
        uint16_t hsiHeadingDeg100 = 0;
        uint16_t hsiObsDeg100 = 0;
        uint16_t hsiHdgBugDeg100 = 0;
        uint8_t  hsiFlags = 0;
        int16_t  hsiCdiDots1000 = 0;
        int16_t  hsiGsDots1000 = 0;
        bool hsiInputWatchActive = false;

        void sendHsiHeadingFrame();
        void sendHsiNavFrame();
        void pollHsiInputWatch();

        String inputBuffer;

        float leftTankLevelKg = 0.;
        float rightTankLevelKg = 0.;

        uint8_t cockpitLightLevel = 0;

        uint16_t rpmValue = 0;
        float odometerHours = 0.;

        float iasKnots = 0.;

        // Altimeter + VSI share one frame, so both commands resend it together.
        float altitudeFt = 0.;
        float vsiFpm = 0.;

        float compassHeadingDeg = 0.;

        // Turn coordinator: one frame carries both needles, so one command
        // sets both (tc<turn>,<slip>).
        float turnRateDeg = 0.;
        float slipDeg = 0.;

        // Rudder watch: prints incoming 0x303 frames until any key is pressed.
        bool rudderWatchActive = false;
        bool rudderWatchPrinted = false; // false = print the next sample unconditionally
        RudderToDcuMessage lastRudderPrinted = {0, 0, 0};

        // Baro watch: same deal for the altimeter's Kollsman knob (0x340).
        bool baroWatchActive = false;
        bool baroWatchPrinted = false; // false = print the next sample unconditionally
        uint16_t lastBaroPrintedInHg100 = 0;


        CAN* canBus;
        uint32_t heartbeat = 0L;
        bool heartbeatLedOn = false;
};
#endif
#endif // BENCHDEBUG_H