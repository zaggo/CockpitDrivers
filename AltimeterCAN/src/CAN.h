#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "Altimeter.h"
#include <CanMessageId.h>
#include <CanNodeId.h>

class CAN : public InstrumentCAN {
    public:
        CAN(Altimeter* altimeter);

        void loop() override;

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        Altimeter* altimeter;

        void sendBaro(uint16_t inHg100);

        // The pot is polled at 20Hz and only sent on a changed inHg*100 value,
        // with an unconditional refresh every 5s so the sim recovers its baro
        // setting after a DCU or plugin restart without anyone touching the knob.
        static const uint32_t kBaroPollIntervalMs = 50;
        static const uint32_t kBaroRefreshMs = 5000;

        uint32_t lastBaroPollMs = 0;
        uint32_t lastBaroSendMs = 0;
        uint16_t lastBaroInHg100 = 0;
        bool baroSentOnce = false;
};

#endif
