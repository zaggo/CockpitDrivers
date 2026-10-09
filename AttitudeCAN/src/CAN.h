#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "AttitudeIndicator.h"

class CAN : public InstrumentCAN {
    public:
        CAN(AttitudeIndicator* attitude)
        : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
          attitude(attitude) {}

    protected:
        bool instrumentBegin() override { return true; }
        void handleFrame(CanMessageId, uint8_t, uint8_t, const uint8_t*) override {}

    private:
        AttitudeIndicator* attitude;
};

#endif
