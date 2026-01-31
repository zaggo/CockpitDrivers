#ifndef DCURECEIVER_H
#define DCURECEIVER_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"

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
        void handleFrame(uint8_t t, uint8_t l, const uint8_t* p);
        void checkMaxAgeResync();

        bool readBytes(uint8_t* dst, size_t n);

        void sendFuelLevel();
        void sendCockpitLightLevel();
        void sendTransponder();

        // RX state machine
        // Fuel Gauge
        uint16_t leftTankLevelKg100 = 0;
        uint16_t rightTankLevelKg100 = 0;
        
        // Cockpit Lights
        uint16_t panelDim1000 = 0;
        uint16_t radioDim1000 = 0;
        uint16_t domeLightDim1000 = 0;

        // Transponder
        uint16_t transponderCode = 10;
        uint8_t transponderMode = 2;
        uint8_t transponderLight = 0;

        // Message metadata for maxAge resync
        MessageMeta fuelLevelMeta;
        MessageMeta cockpitLightMeta;
        MessageMeta transponderMeta;
        
        // Reference to CAN bus
        CAN* canBus;
};
#endif // DCURECEIVER_H