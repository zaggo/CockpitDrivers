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

        // CAN ID error tracking: tracks TX/RX error status per CAN ID.
        // Slots are handed out lazily by setCanIdError() and never freed, so the
        // table has to fit every ID that can ever fail at once. Two sources feed it:
        // one pseudo-ID per monitored instrument node (0x301 + nodeId, every node but
        // the gateway itself — see checkInstrumentHeartbeats) plus one per CAN ID this
        // gateway transmits (8 today: airspeed, altimeterVsi, fuelLevel, lights,
        // odometer, rpm, transponder, gatewayHeartbeat). At 12 the table overflowed
        // on the heartbeats alone, after which every further error was dropped on the
        // floor and the alarm LED stayed dark.
        static constexpr uint8_t kMaxCanIdErrors = kMaxInstrumentNodes + 8;
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