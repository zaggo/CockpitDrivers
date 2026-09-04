#ifndef CAN_H
#define CAN_H
#include <Arduino.h>
#include <BaseCAN.h>
#include "Configuration.h"
#include <CanMessageId.h>
#include <CanNodeId.h>
#include <SerialMessageId.h>
#include "CanIdError.h"
#include "InstrumentLiveness.h"

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

        // Nodes that reported at least once and have since gone quiet.
        // Bit N is node N. Used by BenchDebug and the alarm LED.
        uint16_t silentInstrumentMask() const;

#if BENCHDEBUG
        // Bench console tap. In BENCHDEBUG builds no DCUSender is attached, so decoded
        // rudder frames would otherwise be dropped. Returns true (and clears the slot)
        // when a frame arrived since the last call.
        bool takeRudderSample(RudderToDcuMessage& sample);
#endif

    private:
        uint32_t lastGatewayHeartbeatSendMs = 0;

        // Instrument heartbeat monitoring (nodeId -> last seen)
        static constexpr uint8_t kMaxInstrumentNodes = kLivenessMaxNodes; // 0..15
        uint32_t lastInstrumentHeartbeatMs[kMaxInstrumentNodes] = {0};

        // Liveness lives in two bitmasks instead of one canIdErrors[] entry per
        // node. The old scheme keyed those entries on a fake CAN id (0x301 +
        // nodeId), which aliased real bus ids - 0x303 is both the rudder frame
        // and node 2's pseudo-id - and could fill the table on its own.
        uint16_t instrumentSeenMask = 0;
        uint16_t instrumentAliveMask = 0;

        // CAN ID error tracking: tracks TX/RX error status per CAN ID.
        // Only real bus ids land here now (8 transmitted ids today), so the
        // table no longer has to absorb one entry per monitored node.
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