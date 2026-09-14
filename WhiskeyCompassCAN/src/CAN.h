#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "WhiskeyCompass.h"

class CAN : public InstrumentCAN {
    public:
        CAN(WhiskeyCompass* compass);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        WhiskeyCompass* compass;
};

#endif
