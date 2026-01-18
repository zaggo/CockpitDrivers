#ifndef DCURECEIVER_H
#define DCURECEIVER_H
#include <Arduino.h>
#include "Configuration.h"
#include "CAN.h"

class DCUReceiver {
    public:
        DCUReceiver(CAN* canBus);
        ~DCUReceiver();

        void loop();

    private:
        void sendFuelLevel();
        void sendCockpitLightLevel();
        void handleFrame(uint8_t t, uint8_t l, const uint8_t* p);

        bool readBytes(uint8_t* dst, size_t n);

        uint16_t leftTankLevelKg100 = 0;
        uint16_t rightTankLevelKg100 = 0;
        
        uint16_t panelDim1000 = 0;
        uint16_t radioDim1000 = 0;
        uint16_t domeLightDim1000 = 0;
                
        CAN* canBus;
};
#endif // DCURECEIVER_H