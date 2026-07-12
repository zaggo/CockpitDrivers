#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "Configuration.h"
#include <CanMessageId.h>
#include <CanNodeId.h>
#include "Handbrake.h"

class CAN : public InstrumentCAN {
    public:
        CAN(Handbrake* handbrake);
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
        Handbrake* handbrake;
        unsigned long lastPeriodicSendTime = 0;
        static const unsigned long PERIODIC_SEND_INTERVAL_MS = 2000; // 2 seconds
        void sendHandbrakePosition(uint8_t position);
};
#endif // CAN_H