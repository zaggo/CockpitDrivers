#include "MotionActor.h"
#include "DebugLog.h"
#include <EEPROM.h>

MotionActor::MotionActor() : state(MotionActorState::stopped),
                             kangarooSerial(Serial),
                             K(kangarooSerial)
{
    kangarooSerial.begin(kKangarooBaudRate);
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        const char channelName = static_cast<char>('1' + i);
        actors[i] = new KangarooChannel(K, channelName, kActorAddress);
    }
    DEBUGLOG_PRINT(F("MotionActor initialized with Kangaroo address "));
    DEBUGLOG_PRINTLN(kActorAddress);
}

MotionActor::~MotionActor()
{
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        delete actors[i];
        actors[i] = nullptr;
    }
    DEBUGLOG_PRINTLN(F("MotionActor destructor"));
}

void MotionActor::home()
{
    DEBUGLOG_PRINTLN(F("Homing started"));
    state = MotionActorState::homing;
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        if (actors[i] == nullptr)
        {
            DEBUGLOG_PRINTLN(F("Failed to start actor: channel not initialized"));
            state = MotionActorState::homingFailed;
            return;
        }

        KangarooError error = actors[i]->start();
        if (error != KANGAROO_NO_ERROR && error != KANGAROO_NOT_HOMED)
        {
            DEBUGLOG_PRINT(F("Failed to start actor "));
            DEBUGLOG_PRINT(i + 1);
            DEBUGLOG_PRINT(F(": start() returned error: "));
            DEBUGLOG_PRINTLN(error);
            state = MotionActorState::homingFailed;
            return;
        }
    }

    int32_t ranges[kActorCount] = {0};
    KangarooMonitor monitors[kActorCount];
    KangarooMonitor *monitorList[kActorCount] = {nullptr};
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        monitorList[i] = &monitors[i];
        monitors[i] = actors[i]->home();
    }

    DEBUGLOG_PRINTLN(F("Waiting for homing to complete..."));
    if (waitAll(kActorCount, monitorList))
    {
        for (uint8_t i = 0; i < kActorCount; ++i)
        {
            hardwareMinPosition[i] = actors[i]->getMin().value();
            hardwareMaxPosition[i] = actors[i]->getMax().value();
            ranges[i] = hardwareMaxPosition[i] - hardwareMinPosition[i];
        }

        DEBUGLOG_PRINTLN(F("Homing successful"));
        for (uint8_t i = 0; i < kActorCount; ++i)
        {
            DEBUGLOG_PRINT(F("Hardware Min/Max/Range Actor "));
            DEBUGLOG_PRINT(i + 1);
            DEBUGLOG_PRINTLN(F(":"));
            DEBUGLOG_PRINT(F("Min: "));
            DEBUGLOG_PRINTLN(hardwareMinPosition[i]);
            DEBUGLOG_PRINT(F("Max: "));
            DEBUGLOG_PRINTLN(hardwareMaxPosition[i]);
            DEBUGLOG_PRINT(F("Range: "));
            DEBUGLOG_PRINTLN(ranges[i]);
        }

        for (uint8_t i = 0; i < kActorCount; ++i)
        {
            if (ranges[i] < 8000)
            {
                DEBUGLOG_PRINTLN(F("Homing failed: invalid range"));
                state = MotionActorState::homingFailed;
                return;
            }
        }

        if (!loadLogicalLimitsFromEeprom())
        {
            DEBUGLOG_PRINTLN(F("No logical min/max in EEPROM, using hardware min/max"));
            applyDefaultLogicalLimits();
        }

        if (!validateLogicalLimitsWithinHardware())
        {
            DEBUGLOG_PRINTLN(F("Homing failed: logical min/max outside hardware range"));
            state = MotionActorState::homingFailed;
            return;
        }

        for (uint8_t i = 0; i < kActorCount; ++i)
        {
            DEBUGLOG_PRINT(F("Logical Min/Max Actor "));
            DEBUGLOG_PRINT(i + 1);
            DEBUGLOG_PRINTLN(F(":"));
            DEBUGLOG_PRINT(F("Min: "));
            DEBUGLOG_PRINTLN(logicalMinPosition[i]);
            DEBUGLOG_PRINT(F("Max: "));
            DEBUGLOG_PRINTLN(logicalMaxPosition[i]);
        }

        KangarooMonitor moveMonitors[kActorCount];
        KangarooMonitor *moveMonitorList[kActorCount] = {nullptr};
        for (uint8_t i = 0; i < kActorCount; ++i)
        {
            moveMonitorList[i] = &moveMonitors[i];
            moveMonitors[i] = actors[i]->p(logicalMinPosition[i], max(ranges[i] / 10, 1L));
        }

        DEBUGLOG_PRINTLN(F("Waiting for actors to reach logical minimum..."));
        if (!waitAll(kActorCount, moveMonitorList))
        {
            DEBUGLOG_PRINTLN(F("Homing failed: move to logical minimum timed out or failed"));
            state = MotionActorState::homingFailed;
            return;
        }

        state = MotionActorState::active;
    }
    else
    {
        DEBUGLOG_PRINTLN(F("Homing failed or timed out"));
        state = MotionActorState::homingFailed;
    }
}

void MotionActor::powerDown()
{
    DEBUGLOG_PRINTLN(F("Shutdown"));
    state = MotionActorState::stopped;
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        if (actors[i] != nullptr)
        {
            actors[i]->powerDown();
        }
    }
}

void MotionActor::setDemands(uint16_t demand1, uint16_t demand2)
{
    if (state != MotionActorState::active)
    {
        DEBUGLOG_PRINTLN(F("Cannot set demands: not active"));
        return;
    }
    const uint16_t demands[kActorCount] = {demand1, demand2};
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        DEBUGLOG_PRINT(F("Logical Min/Max Actor "));
        DEBUGLOG_PRINT(i + 1);
        DEBUGLOG_PRINTLN(F(":"));
        DEBUGLOG_PRINT(F("Min: "));
        DEBUGLOG_PRINTLN(logicalMinPosition[i]);
        DEBUGLOG_PRINT(F("Max: "));
        DEBUGLOG_PRINTLN(logicalMaxPosition[i]);

        int32_t pos = map(demands[i], 0, 0xffff, logicalMinPosition[i], logicalMaxPosition[i]);
        DEBUGLOG_PRINT(F("Set Actor "));
        DEBUGLOG_PRINT(i + 1);
        DEBUGLOG_PRINT(F(" pos "));
        DEBUGLOG_PRINTLN(String(static_cast<int>(pos)));
        actors[i]->p(pos);
    }
}

void MotionActor::calibrationMove(uint8_t channel, uint16_t positionPercent)
{
    if (state != MotionActorState::active)
    {
        DEBUGLOG_PRINTLN(F("Cannot calibrate: actor not active"));
        return;
    }

    if (channel >= kActorCount)
    {
        DEBUGLOG_PRINTLN(F("Invalid actor channel for calibration move"));
        return;
    }

    const int32_t range = hardwareMaxPosition[channel] - hardwareMinPosition[channel];
    const int32_t targetPosition = map(positionPercent, 0, 0xFFFF, hardwareMinPosition[channel], hardwareMaxPosition[channel]);
    const int32_t speed = max(range / 10, 1L);

    DEBUGLOG_PRINT(F("Calibration move channel "));
    DEBUGLOG_PRINT(channel);
    DEBUGLOG_PRINT(F(" to percent "));
    DEBUGLOG_PRINT(positionPercent);
    DEBUGLOG_PRINT(F(" -> position "));
    DEBUGLOG_PRINTLN(targetPosition);

    KangarooChannel* motor = channelForIndex(channel);
    if (motor == nullptr)
    {
        return;
    }
    motor->p(targetPosition, speed);
}

void MotionActor::saveLogicalMin(uint8_t channel)
{
    if (state != MotionActorState::active)
    {
        DEBUGLOG_PRINTLN(F("Cannot save logical min: actor not active"));
        return;
    }

    int32_t currentPosition = 0;
    if (!readCurrentPosition(channel, currentPosition))
    {
        return;
    }

    if (currentPosition > logicalMaxPosition[channel])
    {
        DEBUGLOG_PRINTLN(F("Cannot save logical min above logical max"));
        return;
    }

    logicalMinPosition[channel] = currentPosition;
    saveLogicalLimitsToEeprom();

    DEBUGLOG_PRINT(F("Saved logical min for channel "));
    DEBUGLOG_PRINT(channel);
    DEBUGLOG_PRINT(F(": "));
    DEBUGLOG_PRINTLN(currentPosition);
}

void MotionActor::saveLogicalMax(uint8_t channel)
{
    if (state != MotionActorState::active)
    {
        DEBUGLOG_PRINTLN(F("Cannot save logical max: actor not active"));
        return;
    }

    int32_t currentPosition = 0;
    if (!readCurrentPosition(channel, currentPosition))
    {
        return;
    }

    if (currentPosition < logicalMinPosition[channel])
    {
        DEBUGLOG_PRINTLN(F("Cannot save logical max below logical min"));
        return;
    }

    logicalMaxPosition[channel] = currentPosition;
    saveLogicalLimitsToEeprom();

    DEBUGLOG_PRINT(F("Saved logical max for channel "));
    DEBUGLOG_PRINT(channel);
    DEBUGLOG_PRINT(F(": "));
    DEBUGLOG_PRINTLN(currentPosition);
}

KangarooChannel* MotionActor::channelForIndex(uint8_t channel)
{
    if (channel >= kActorCount)
    {
        return nullptr;
    }
    return actors[channel];
}

bool MotionActor::readCurrentPosition(uint8_t channel, int32_t& position)
{
    if (channel >= kActorCount)
    {
        DEBUGLOG_PRINTLN(F("Invalid actor channel"));
        return false;
    }

    KangarooChannel* motor = channelForIndex(channel);
    if (motor == nullptr)
    {
        DEBUGLOG_PRINTLN(F("No motor for actor channel"));
        return false;
    }

    KangarooStatus current = motor->getP();
    if (!current.valid() || !current.ok())
    {
        DEBUGLOG_PRINTLN(F("Failed to read current motor position"));
        return false;
    }

    position = current.value();
    return true;
}

bool MotionActor::loadLogicalLimitsFromEeprom()
{
    StoredLogicalLimits stored;
    EEPROM.get(kEepromAddress, stored);

    if (stored.magic != kEepromMagic ||
        stored.version != kEepromVersion ||
        stored.size != sizeof(StoredLogicalLimits))
    {
        return false;
    }

    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        logicalMinPosition[i] = stored.min[i];
        logicalMaxPosition[i] = stored.max[i];
    }
    return true;
}

void MotionActor::saveLogicalLimitsToEeprom()
{
    StoredLogicalLimits stored;
    stored.magic = kEepromMagic;
    stored.version = kEepromVersion;
    stored.size = sizeof(StoredLogicalLimits);

    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        stored.min[i] = logicalMinPosition[i];
        stored.max[i] = logicalMaxPosition[i];
    }

    EEPROM.put(kEepromAddress, stored);
}

void MotionActor::applyDefaultLogicalLimits()
{
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        logicalMinPosition[i] = hardwareMinPosition[i];
        logicalMaxPosition[i] = hardwareMaxPosition[i];
    }
}

bool MotionActor::validateLogicalLimitsWithinHardware() const
{
    for (uint8_t i = 0; i < kActorCount; ++i)
    {
        if (logicalMinPosition[i] < hardwareMinPosition[i] ||
            logicalMaxPosition[i] > hardwareMaxPosition[i] ||
            logicalMinPosition[i] > logicalMaxPosition[i])
        {
            return false;
        }
    }
    return true;
}