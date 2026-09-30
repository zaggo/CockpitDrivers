#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <InstrumentCAN.h>
#include "HSI.h"
#include <CanMessageId.h>
#include <CanNodeId.h>

class CAN : public InstrumentCAN {
    public:
        CAN(HSI* hsi);

        void loop() override;

        // Instrument -> Gateway, onChange only. Never resent: the payload is a
        // delta and a replay would turn the knob twice.
        void sendHsiInput(uint8_t control, uint8_t action, int16_t delta, uint8_t pushState);

    protected:
        bool instrumentBegin() override;
        void onStartupFail() override;
        void handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t* data) override;
        void onGatewayHeartbeatTimeout() override;
        void onGatewayHeartbeatDiscovered() override;

    private:
        HSI* hsi;

        // Homing failure is signalled on the panel light, the way
        // WhiskeyCompassCAN does, and retried - otherwise a dead hall sensor
        // leaves the instrument frozen until someone power-cycles it.
        static const uint32_t kHomingBlinkMs = 400;
        static const uint32_t kHomingRetryMs = 10000;
        uint32_t lastBlinkMs = 0;
        uint32_t homingFailedAtMs = 0;
        bool blinkOn = false;

        // 0 is a legitimate dimmer setting, so it cannot double as "unknown".
        // Without this flag a reconnect with the dimmer at zero flashes the
        // panel to full until the gateway's next lights resync.
        uint8_t panelPwm = 0;
        bool panelPwmKnown = false;
};

#endif
