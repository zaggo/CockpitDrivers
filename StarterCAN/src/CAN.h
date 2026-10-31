#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "Configuration.h"
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "Starter.h"

class CAN : public InstrumentCAN {
    public:
        CAN(Starter* starter);
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
        Starter* starter;
        unsigned long lastPeriodicSendTime = 0;
        // Cluster-input block rule: absolute inputs resend unconditionally every
        // 5s, so the sim recovers the switch position after a DCU/plugin restart.
        static const unsigned long PERIODIC_SEND_INTERVAL_MS = 5000;
        void sendStarterPosition(uint8_t position);
};
#endif // CAN_H
