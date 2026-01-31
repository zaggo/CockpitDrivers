#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "LCD.h"
#include "Configuration.h"
#include <CanMessageId.h>
#include <CanNodeId.h>

class CAN : public InstrumentCAN {
    public:
        CAN(LCD* lcd);

    protected:
        // Override from InstrumentCAN
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        LCD* lcd;
};

#endif