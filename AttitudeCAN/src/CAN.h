#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "AttitudeIndicator.h"

class CAN : public InstrumentCAN {
    public:
        CAN(AttitudeIndicator* attitude);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        AttitudeIndicator* attitude;

        // Last panel brightness from a lights frame. 0 is a legitimate dimmer
        // setting, so "known" is tracked separately from the value.
        uint8_t panelPwm = 0;
        bool panelPwmKnown = false;
};

#endif
