#ifndef MOTIONACTOR_H
#define MOTIONACTOR_H
#include <Arduino.h>
#include <Kangaroo.h>
#include "Configuration.h"
#include <MotionNodeId.h>

class MotionActor {
    public:
        MotionActor();
        ~MotionActor();

        void home();
        void setDemands(uint16_t demand1, uint16_t demand2);
        // Profiled move: one fire-and-forget p(target, speed) per channel, speed
        // chosen so BOTH channels arrive after durationMs (coordinated arrival).
        // Used for arm/disarm; runs in streaming mode like setDemands.
        void gotoDemands(uint16_t demand1, uint16_t demand2, uint16_t durationMs);
        void calibrationMove(uint8_t channel, uint16_t positionPercent);
        void saveLogicalMin(uint8_t channel);
        void saveLogicalMax(uint8_t channel);
        void powerDown();

        MotionActorState state;

#if MOTION_TESTBENCH
        // Test-bench access to the raw Kangaroo channels and calibrated limits;
        // the bench issues its own speed-limited moves to compare strategies.
        KangarooChannel* testChannel(uint8_t channel) { return channelForIndex(channel); }
        int32_t testLogicalMin(uint8_t channel) const { return logicalMinPosition[channel]; }
        int32_t testLogicalMax(uint8_t channel) const { return logicalMaxPosition[channel]; }
#endif
    private:
        static constexpr uint8_t kActorCount = 2;

        HardwareSerial& kangarooSerial;
        KangarooSerial  K;
        KangarooChannel* actors[kActorCount] = {nullptr, nullptr};

        static constexpr int kEepromAddress = 0;
        static constexpr uint16_t kEepromMagic = 0x4D41;
        static constexpr uint8_t kEepromVersion = 1;

        struct StoredLogicalLimits
        {
            uint16_t magic;
            uint8_t version;
            uint8_t size;
            int32_t min[kActorCount];
            int32_t max[kActorCount];
        };

        int32_t hardwareMinPosition[kActorCount] = {0};
        int32_t hardwareMaxPosition[kActorCount] = {0};
        int32_t logicalMinPosition[kActorCount] = {0};
        int32_t logicalMaxPosition[kActorCount] = {0};

        // Demand-to-demand speed shaping (see setDemands): remember the previously
        // commanded position and when it was commanded, so each p() can carry a
        // speed limit that makes the actuator glide between targets instead of
        // snapping at the Kangaroo's maximum speed.
        int32_t lastCommandedPosition[kActorCount] = {0};
        bool haveLastCommanded = false;
        unsigned long lastDemandTimestampMs = 0;

        KangarooChannel* channelForIndex(uint8_t channel);
        void setStreaming(bool enabled);
        bool readCurrentPosition(uint8_t channel, int32_t& position);
        bool loadLogicalLimitsFromEeprom();
        void saveLogicalLimitsToEeprom();
        void applyDefaultLogicalLimits();
        bool validateLogicalLimitsWithinHardware() const;
};

#endif // MOTIONACTOR_H