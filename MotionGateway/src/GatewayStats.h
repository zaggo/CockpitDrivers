#ifndef GATEWAYSTATS_H
#define GATEWAYSTATS_H
#include <Arduino.h>
#include "DebugLog.h"

// Serial->CAN timing instrumentation. One summary line every 5 s via DEBUGLOG
// (Serial1 in the production build), so the USB link to the plugin stays clean:
//
//   GS,frames,<dtMin>,<dtMean>,<dtMax>,<b<10>,<b10-20>,<b20-35>,<b35-50>,<b>50>,
//      resync,crMiss,discard,sendMaxUs,txFail
//
// dt = interarrival of complete demand frames in ms. Buckets count frames.
// resync = bytes skipped hunting for 'B', crMiss = bytes eaten waiting for the
// terminating CR (framing loss), discard = bytes dropped in mode0.
struct GatewayStats
{
    uint32_t frameCount = 0;
    uint32_t lastFrameMs = 0;
    uint32_t dtMin = 0xFFFFFFFF;
    uint32_t dtMax = 0;
    uint32_t dtSum = 0;
    uint32_t buckets[5] = {0}; // <10, 10-20, 20-35, 35-50, >50 ms
    uint32_t resyncBytes = 0;
    uint32_t crMissBytes = 0;
    uint32_t discardedBytes = 0;
    uint32_t sendMaxUs = 0;

    void noteFrame(uint32_t nowMs)
    {
        if (lastFrameMs != 0)
        {
            const uint32_t dt = nowMs - lastFrameMs;
            if (dt < dtMin) dtMin = dt;
            if (dt > dtMax) dtMax = dt;
            dtSum += dt;
            if (dt < 10)       buckets[0]++;
            else if (dt < 20)  buckets[1]++;
            else if (dt < 35)  buckets[2]++;
            else if (dt < 50)  buckets[3]++;
            else               buckets[4]++;
        }
        lastFrameMs = nowMs;
        ++frameCount;
    }

    void noteSend(uint32_t durUs)
    {
        if (durUs > sendMaxUs) sendMaxUs = durUs;
    }

    void print(uint16_t txFailures) const
    {
#if DEBUGLOG_ENABLE
        const uint32_t intervals = frameCount > 0 ? frameCount - 1 : 0;
        const uint32_t dtMean = intervals > 0 ? dtSum / intervals : 0;
        DEBUGLOG_PRINTLN(String(F("GS,")) + frameCount + "," +
                         (intervals > 0 ? dtMin : 0) + "," + dtMean + "," + dtMax + "," +
                         buckets[0] + "," + buckets[1] + "," + buckets[2] + "," +
                         buckets[3] + "," + buckets[4] + "," +
                         resyncBytes + "," + crMissBytes + "," + discardedBytes + "," +
                         sendMaxUs + "," + txFailures);
#else
        (void)txFailures;
#endif
    }

    void reset()
    {
        // Keep lastFrameMs so the first dt after a reset stays valid.
        frameCount = 0;
        dtMin = 0xFFFFFFFF;
        dtMax = 0;
        dtSum = 0;
        for (uint8_t i = 0; i < 5; ++i) buckets[i] = 0;
        resyncBytes = 0;
        crMissBytes = 0;
        discardedBytes = 0;
        sendMaxUs = 0;
    }
};

#endif // GATEWAYSTATS_H
