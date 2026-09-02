#ifndef DCURECEIVER_H
#define DCURECEIVER_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"
#include "DCUSender.h"
#include <SerialMessageId.h>
#include "SerialFrameParser.h"

// Message metadata for maxAge resync
struct MessageMeta {
    unsigned long lastSendTimestamp;
    unsigned long maxAgeMs;
};

class DCUReceiver {
    public:
        DCUReceiver(CAN* canBus);
        ~DCUReceiver();

        void loop();

    private:        
        void handleFrame(MessageType t, uint8_t l, const uint8_t* p);
        void checkMaxAgeResync();

        void sendFuelLevel();
        void sendCockpitLightLevel();
        void sendTransponder();
        void sendRpm();
        void sendOdometer();
        void sendAirspeed();
        void sendAltimeterVsi();

        // RX state machine
        // Fuel Gauge
        uint16_t leftTankLevelKg100 = 0;
        uint16_t rightTankLevelKg100 = 0;
        
        // Cockpit Lights
        uint16_t panelDim1000 = 0;
        uint16_t radioDim1000 = 0;
        uint16_t domeLightDim1000 = 0;

        // Transponder
        uint16_t transponderCode = 0;
        uint8_t transponderMode = 3;
        uint8_t transponderLight = 0;

        // RPM Gauge
        uint16_t rpmValue = 0;

        // Odometer (RPM Gauge)
        uint8_t tachHrs1000 = 0;
        uint8_t tachHrs100 = 0;
        uint8_t tachHrs10 = 0;
        uint8_t tachHrs1 = 0;
        uint8_t tachHrsTenths = 0;
        uint16_t tachHrsHundredths100 = 0;

        // Airspeed Indicator (ASI)
        uint16_t iasKts10 = 0;
        uint16_t tasKts10 = 0;

        // Altimeter + Vertical Speed Indicator (share CAN 0x102)
        int32_t altitudeFt = 0;
        int16_t vsiFpm = 0;

        // Message metadata for maxAge resync
        MessageMeta fuelLevelMeta;
        MessageMeta cockpitLightMeta;
        MessageMeta transponderMeta;
        MessageMeta rpmMeta;
        MessageMeta odometerMeta;
        MessageMeta airspeedMeta;
        MessageMeta altimeterVsiMeta;

        // Reference to CAN bus
        CAN* canBus;
        
        // DCUSender instance for sending data back to DCUProvider Plugin
        DCUSender* dcuSender;

        // Serial framing parser (0xAA 0x55 TYPE LEN PAYLOAD...)
        SerialFrameParser frameParser;
};
#endif // DCURECEIVER_H