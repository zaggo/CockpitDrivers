#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <BaseCAN.h>
#include "Configuration.h"
#include <CanMessageId.h>
#include <CanNodeId.h>
#include <SerialMessageId.h>
#include "CanIdError.h"

// Forward declaration
class DCUSender;

class CAN : public BaseCAN {
    public:
        CAN();
        ~CAN();

        bool begin() override;

        void loop();

        void sendMessage(CanMessageId id, uint8_t len, byte* data);
        
        void setDCUSender(DCUSender* sender);

#if BENCHDEBUG
        // Bench console tap. In BENCHDEBUG builds no DCUSender is attached, so decoded
        // rudder frames would otherwise be dropped. Returns true (and clears the slot)
        // when a frame arrived since the last call.
        bool takeRudderSample(RudderToDcuMessage& sample);
#endif

    private:
        uint32_t lastGatewayHeartbeatSendMs = 0;

        // Instrument heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxInstrumentNodes = 16; // 0..15
        uint32_t lastInstrumentHeartbeatMs[kMaxInstrumentNodes] = {0};
        bool instrumentAlive[kMaxInstrumentNodes] = {false};

        // CAN ID error tracking: tracks TX/RX error status per CAN ID
        static constexpr uint8_t kMaxCanIdErrors = 12;
        CanIdError canIdErrors[kMaxCanIdErrors];
        uint8_t canIdErrorCount = 0;

        // Reference to DCUSender for sending messages back to DCUProvider Plugin
        DCUSender* dcuSender = nullptr;

#if BENCHDEBUG
        // Last decoded rudder frame, drained by takeRudderSample().
        RudderToDcuMessage rudderSample = {0, 0, 0};
        bool rudderSampleValid = false;
#endif

        // Handle incoming Serial Message frames
        void updateInstrumentHeartbeat(uint8_t len, const uint8_t* data);
        void updateTransponder(uint8_t len, const uint8_t* data);
        void updateHandbrake(uint8_t len, const uint8_t* data);
        void updateRudder(uint8_t len, const uint8_t* data);

        void sendGatewayHeartbeat();
        void checkInstrumentHeartbeats();
        void updateAlarmLED();
        void clearCanIdError(uint16_t canId, CanErrorType errorType = CanErrorType::NONE);
        void setCanIdError(uint16_t canId, CanErrorType errorType);

        void handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t* data);
};

#endif