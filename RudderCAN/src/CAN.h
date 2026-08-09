#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "Configuration.h"
#include "Rudder.h"

class CAN : public InstrumentCAN {
    public:
        CAN(Rudder* rudder);
        ~CAN();

        void loop() override;

    protected:
        // Override from InstrumentCAN
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        Rudder* rudder;
        uint32_t lastSendTime = 0;

        // The rudder axis moves continuously, so unthrottled send-on-change would
        // flood the bus. 50Hz matches the cadence the other axes run at.
        static const uint32_t kMinSendIntervalMs = 20;
        // Fallback so the DCU picks up the current position even if it (re)starts
        // while the pedals are standing still.
        static const uint32_t kPeriodicSendIntervalMs = 2000;

        void sendRudderState(const RudderState& state);
};
#endif // CAN_H
