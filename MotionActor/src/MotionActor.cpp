#include "MotionActor.h"
#include "DebugLog.h"

MotionActor::MotionActor() : state(MotionActorState::stopped),
                             kangarooSerial(Serial),
                             K(kangarooSerial),
                             actor1(K, 1, kActorAddress),
                             actor2(K, 2, kActorAddress)
{
    kangarooSerial.begin(kKangarooBaudRate);
    DEBUGLOG_PRINTLN(F("MotionActor constructor"));
}

MotionActor::~MotionActor()
{
    DEBUGLOG_PRINTLN(F("MotionActor destructor"));
}

void MotionActor::home()
{
    state = MotionActorState::homing;
    actor1.start();
    actor2.start();

    KangarooMonitor monitor1, monitor2;
    KangarooMonitor *monitorList[2] = {&monitor1, &monitor2};
    monitor1 = actor1.home();
    monitor2 = actor2.home();
    if (waitAll(2, monitorList))
    {
        minPosition1 = actor1.getMin().value();
        maxPosition1 = actor1.getMax().value();
        int32_t range1 = maxPosition1 - minPosition1;

        minPosition2 = actor2.getMin().value();
        maxPosition2 = actor2.getMax().value();
        int32_t range2 = maxPosition2 - minPosition2;

        // Move away from the end positions by 5%, to avoid triggering the end switches
        int32_t margin = range1 / 20;
        range1 -= margin;
        minPosition1 += margin / 2;
        maxPosition1 -= margin / 2;

        int32_t margin2 = range2 / 20;
        range2 -= margin2;
        minPosition2 += margin2 / 2;
        maxPosition2 -= margin2 / 2;

        if (minPosition1 < 200 || maxPosition1 < 10000 || range1 < 8000 ||
            minPosition2 < 200 || maxPosition2 < 10000 || range2 < 8000)
        {
            DEBUGLOG_PRINTLN(F("Homing failed: invalid range"));
            state = MotionActorState::homingFailed;
            return;
        }

        DEBUGLOG_PRINTLN(F("Homing successful"));
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
    actor1.powerDown();
    actor2.powerDown();
}

void MotionActor::setDemands(uint16_t demand1, uint16_t demand2)
{
    if (state != MotionActorState::active)
    {
        DEBUGLOG_PRINTLN(F("Cannot set demands: not active"));
        return;
    }

    int32_t pos = map(demand1, 0, 0xffff, minPosition1,  maxPosition1);
    actor1.p(pos);
    pos = map(demand2, 0, 0xffff, minPosition2,  maxPosition2);
    actor2.p(pos);
}