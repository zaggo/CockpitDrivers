#include "TestBench.h"

#if MOTION_TESTBENCH

#include <BaseCAN.h>
#include <MotionMessageId.h>
#include "DebugLog.h"

TestBench::Sample TestBench::samples[TestBench::kMaxSamples];

TestBench::TestBench(MotionActor *motionActor, BaseCAN *canBus)
    : motionActor(motionActor), canBus(canBus)
{
}

void TestBench::startTest(uint8_t len, const uint8_t *data)
{
    if (len < 8 || isRunning() || state == State::dumping)
    {
        return;
    }
    if (motionActor->state != MotionActorState::active)
    {
        sendStatus(3);
        return;
    }

    strategy = data[1];
    rateHz = constrain(data[2], (uint8_t)1, (uint8_t)100);
    amplitudePct = constrain(data[3], (uint8_t)1, (uint8_t)100);
    channelMask = data[4] & 0x03;
    sampleMode = data[5];
    durationSec = constrain(data[6], (uint8_t)1, (uint8_t)60);
    param = max(data[7], (uint8_t)1);

    if (channelMask == 0)
    {
        channelMask = 0x01;
    }
    firstActiveChannel = (channelMask & 0x01) ? 0 : 1;

    // Strategy 0 only makes sense with a position series; the wire is otherwise idle.
    if (strategy == 0)
    {
        sampleMode = 1;
    }

    sampleKind = kSampleNone;
    if (sampleMode == 1 || sampleMode == 3)
    {
        sampleKind = kSamplePosition;
    }
    else if (sampleMode == 2)
    {
        sampleKind = kSampleTiming;
    }

    sampleCount = 0;
    cyclesDone = 0;
    missedCycles = 0;
    maxCmdDurationUs10 = 0;
    resultState = 0;
    haveLastTarget = false;
    periodUs = 1000000UL / rateHz;

    if (strategy == 9)
    {
        // Passthrough: instrument the production demand path, no generator, no
        // preposition - the upstream stream (gateway/plugin) owns the motion.
        runStartMs = millis();
        nextPositionSampleMs = runStartMs;
        state = State::running;
        return;
    }

    // Glide all active channels to the window start so every run begins identically.
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        if (!(channelMask & (1 << ch)))
        {
            continue;
        }
        KangarooChannel *motor = motionActor->testChannel(ch);
        if (motor == nullptr)
        {
            sendStatus(3);
            return;
        }
        const int32_t range = motionActor->testLogicalMax(ch) - motionActor->testLogicalMin(ch);
        motor->p(windowMin(ch), max(range / 5, 1L));
    }
    prepositionStartMs = millis();
    nextPrepositionPollMs = prepositionStartMs;
    state = State::prepositioning;
}

void TestBench::abortTest()
{
    if (isRunning())
    {
        finishRun(2);
    }
    else if (state == State::dumping)
    {
        state = State::done;
    }
}

void TestBench::requestDump()
{
    if (state != State::done)
    {
        return;
    }
    sendDumpHeader();
    dumpIndex = 0;
    nextDumpFrameMs = millis() + kDumpFrameIntervalMs;
    state = State::dumping;
}

void TestBench::loop()
{
    switch (state)
    {
    case State::prepositioning:
        tickPreposition();
        break;
    case State::running:
        tickRun();
        break;
    case State::dumping:
        tickDump();
        break;
    default:
        break;
    }
}

void TestBench::tickPreposition()
{
    const uint32_t now = millis();
    if ((int32_t)(now - nextPrepositionPollMs) < 0)
    {
        return;
    }
    nextPrepositionPollMs = now + kPrepositionPollIntervalMs;

    if (now - prepositionStartMs > kPrepositionTimeoutMs)
    {
        finishRun(3);
        return;
    }

    int32_t position = 0;
    if (!readPosition(firstActiveChannel, position))
    {
        return; // transient read failure, retry on next poll
    }
    const int32_t range = motionActor->testLogicalMax(firstActiveChannel) -
                          motionActor->testLogicalMin(firstActiveChannel);
    if (abs(position - windowMin(firstActiveChannel)) > max(range / 50, 4L))
    {
        return; // still traveling
    }

    // Arrived at the window start - begin the measured run.
    if (strategy == 3 || strategy == 4)
    {
        setStreaming(true);
    }

    runStartMs = millis();
    legStartMs = runStartMs;
    movingUp = true;
    nextTickUs = micros();
    nextPositionSampleMs = runStartMs;
    lastTickMs = runStartMs;
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        legStartTarget[ch] = windowMin(ch);
        lastTarget[ch] = windowMin(ch);
    }
    haveLastTarget = true;

    if (strategy == 0)
    {
        // One long move at a speed that traverses the window in kLegDurationMs -
        // the Kangaroo runs its own continuous ramp (the smooth homing baseline).
        for (uint8_t ch = 0; ch < 2; ++ch)
        {
            if (!(channelMask & (1 << ch)))
            {
                continue;
            }
            const int32_t span = windowMax(ch) - windowMin(ch);
            const int32_t speed = max((span * 1000L) / (int32_t)kLegDurationMs, 1L);
            motionActor->testChannel(ch)->p(windowMax(ch), speed);
        }
    }

    state = State::running;
}

void TestBench::tickRun()
{
    const uint32_t nowMs = millis();

    const bool durationElapsed = (nowMs - runStartMs) >= (uint32_t)durationSec * 1000UL;
    const bool bufferFull = (sampleKind != kSampleNone) && (sampleCount >= kMaxSamples);
    if (durationElapsed || bufferFull)
    {
        finishRun(1);
        return;
    }

    if (strategy == 0)
    {
        samplePositionIfDue(false);
        return;
    }
    if (strategy == 9)
    {
        // Samples arrive via noteDemandApplied(); nothing to generate here.
        return;
    }

    // Step-stream strategies: absolute-deadline tick scheduling.
    const uint32_t nowUs = micros();
    if ((int32_t)(nowUs - nextTickUs) < 0)
    {
        return;
    }
    nextTickUs += periodUs;
    if ((int32_t)(nowUs - nextTickUs) >= 0)
    {
        // Fell more than a full period behind (blocking Kangaroo round-trips):
        // count the shortfall and resynchronize instead of bursting catch-up moves.
        missedCycles += (uint16_t)((nowUs - nextTickUs) / periodUs + 1);
        nextTickUs = nowUs + periodUs;
    }

    // Triangle generator: traverse the window in kLegDurationMs per leg.
    uint32_t legElapsed = nowMs - legStartMs;
    if (legElapsed >= kLegDurationMs)
    {
        movingUp = !movingUp;
        legStartMs = nowMs;
        legElapsed = 0;
        for (uint8_t ch = 0; ch < 2; ++ch)
        {
            legStartTarget[ch] = movingUp ? windowMin(ch) : windowMax(ch);
        }
    }

    int32_t targets[2];
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        const int32_t from = movingUp ? windowMin(ch) : windowMax(ch);
        const int32_t to = movingUp ? windowMax(ch) : windowMin(ch);
        targets[ch] = from + (int32_t)(((int64_t)(to - from) * (int32_t)legElapsed) / (int32_t)kLegDurationMs);
    }

    // Issue the per-channel moves, speed-limited per strategy.
    const uint32_t dtMs = nowMs - lastTickMs;
    lastTickMs = nowMs;
    const uint32_t cmdStartUs = micros();
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        if (!(channelMask & (1 << ch)))
        {
            continue;
        }
        int32_t delta = targets[ch] - lastTarget[ch];
        if (delta < 0)
        {
            delta = -delta;
        }
        if (delta == 0)
        {
            continue;
        }

        int32_t speed;
        if (strategy == 1 || strategy == 3)
        {
            // Production algorithm (MotionActor::setDemands): clamp dt to 10..100 ms,
            // 20% headroom, floored at range/5 per second.
            uint32_t dt = dtMs;
            if (dt < 10) dt = 10;
            if (dt > 100) dt = 100;
            const int32_t range = motionActor->testLogicalMax(ch) - motionActor->testLogicalMin(ch);
            speed = (delta * 1200L) / (int32_t)dt;
            const int32_t minSpeed = max(range / 5, 1L);
            if (speed < minSpeed) speed = minSpeed;
        }
        else
        {
            // Exact matching: arrive precisely when the next step is due, so
            // consecutive moves chain without a dwell.
            uint32_t dt = dtMs;
            if (dt < 5) dt = 5;
            if (dt > 250) dt = 250;
            speed = max((delta * 1000L) / (int32_t)dt, 1L);
        }

        motionActor->testChannel(ch)->p(targets[ch], speed);
        lastTarget[ch] = targets[ch];
    }
    const uint32_t cmdDurUs = micros() - cmdStartUs;
    const uint16_t dur10 = (uint16_t)min(cmdDurUs / 10UL, 65535UL);
    if (dur10 > maxCmdDurationUs10)
    {
        maxCmdDurationUs10 = dur10;
    }
    if (cyclesDone < 0xFFFF)
    {
        ++cyclesDone;
    }

    if (sampleMode == 2)
    {
        recordSample(dur10);
    }
    else if (sampleMode == 3 && (cyclesDone % param) == 0)
    {
        samplePositionIfDue(true);
    }
}

void TestBench::noteDemandApplied(uint32_t cmdDurationUs)
{
    if (!isPassthroughActive())
    {
        return;
    }

    const uint16_t dur10 = (uint16_t)min(cmdDurationUs / 10UL, 65535UL);
    if (dur10 > maxCmdDurationUs10)
    {
        maxCmdDurationUs10 = dur10;
    }
    if (cyclesDone < 0xFFFF)
    {
        ++cyclesDone;
    }

    if (sampleMode == 2)
    {
        recordSample(dur10);
    }
    else if (sampleMode == 3 && (cyclesDone % param) == 0)
    {
        samplePositionIfDue(true);
    }
}

void TestBench::samplePositionIfDue(bool force)
{
    const uint32_t now = millis();
    if (!force && (int32_t)(now - nextPositionSampleMs) < 0)
    {
        return;
    }
    nextPositionSampleMs = now + kPositionSampleIntervalMs;

    int32_t position = 0;
    if (!readPosition(firstActiveChannel, position))
    {
        return;
    }
    recordSample(normalizePosition(firstActiveChannel, position));
}

void TestBench::recordSample(uint16_t value)
{
    if (sampleCount >= kMaxSamples)
    {
        return;
    }
    samples[sampleCount].tOffsetMs = (uint16_t)(millis() - runStartMs);
    samples[sampleCount].value = value;
    ++sampleCount;
}

bool TestBench::readPosition(uint8_t channel, int32_t &position)
{
    KangarooChannel *motor = motionActor->testChannel(channel);
    if (motor == nullptr)
    {
        return false;
    }
    KangarooStatus status = motor->getP();
    if (!status.valid() || !status.ok())
    {
        return false;
    }
    position = status.value();
    return true;
}

uint16_t TestBench::normalizePosition(uint8_t channel, int32_t position) const
{
    const int32_t minPos = motionActor->testLogicalMin(channel);
    const int32_t range = motionActor->testLogicalMax(channel) - minPos;
    if (range <= 0)
    {
        return 0;
    }
    int64_t scaled = ((int64_t)(position - minPos) * 65535L) / range;
    if (scaled < 0) scaled = 0;
    if (scaled > 65535) scaled = 65535;
    return (uint16_t)scaled;
}

int32_t TestBench::windowMin(uint8_t channel) const
{
    const int32_t minPos = motionActor->testLogicalMin(channel);
    const int32_t range = motionActor->testLogicalMax(channel) - minPos;
    const int32_t center = minPos + range / 2;
    return center - (int32_t)(((int64_t)range * amplitudePct) / 200);
}

int32_t TestBench::windowMax(uint8_t channel) const
{
    const int32_t minPos = motionActor->testLogicalMin(channel);
    const int32_t range = motionActor->testLogicalMax(channel) - minPos;
    const int32_t center = minPos + range / 2;
    return center + (int32_t)(((int64_t)range * amplitudePct) / 200);
}

void TestBench::setStreaming(bool enabled)
{
    for (uint8_t ch = 0; ch < 2; ++ch)
    {
        KangarooChannel *motor = motionActor->testChannel(ch);
        if (motor != nullptr)
        {
            motor->streaming(enabled);
        }
    }
}

void TestBench::finishRun(uint8_t result)
{
    if (strategy == 3 || strategy == 4)
    {
        setStreaming(false);
    }
    resultState = result;
    state = State::done;
    sendStatus(result);
}

void TestBench::sendStatus(uint8_t result)
{
    byte data[8] = {0};
    data[0] = static_cast<uint8_t>(kNodeId);
    data[1] = result;
    data[2] = (uint8_t)(cyclesDone >> 8);
    data[3] = (uint8_t)(cyclesDone & 0xFF);
    data[4] = (uint8_t)(maxCmdDurationUs10 >> 8);
    data[5] = (uint8_t)(maxCmdDurationUs10 & 0xFF);
    data[6] = (uint8_t)(missedCycles >> 8);
    data[7] = (uint8_t)(missedCycles & 0xFF);
    canBus->sendMessage(static_cast<uint16_t>(MotionMessageId::testStatus), 8, data);
}

void TestBench::sendDumpHeader()
{
    byte data[8] = {0};
    data[0] = static_cast<uint8_t>(kNodeId);
    data[1] = strategy;
    data[2] = sampleKind;
    data[3] = (uint8_t)(sampleCount >> 8);
    data[4] = (uint8_t)(sampleCount & 0xFF);
    data[5] = rateHz;
    data[6] = channelMask;
    data[7] = resultState;
    canBus->sendMessage(static_cast<uint16_t>(MotionMessageId::testDumpHeader), 8, data);
}

void TestBench::tickDump()
{
    const uint32_t now = millis();
    if ((int32_t)(now - nextDumpFrameMs) < 0)
    {
        return;
    }
    nextDumpFrameMs = now + kDumpFrameIntervalMs;

    if (dumpIndex >= sampleCount)
    {
        // Footer: repeat the aggregate stats so the console log ends with them.
        state = State::done;
        sendStatus(resultState);
        return;
    }

    const Sample &s = samples[dumpIndex];
    byte data[8] = {0};
    data[0] = static_cast<uint8_t>(kNodeId);
    data[1] = (uint8_t)(dumpIndex & 0xFF); // seq, wraps at 256
    data[2] = (uint8_t)(s.tOffsetMs >> 8);
    data[3] = (uint8_t)(s.tOffsetMs & 0xFF);
    data[4] = (uint8_t)(s.value >> 8);
    data[5] = (uint8_t)(s.value & 0xFF);
    canBus->sendMessage(static_cast<uint16_t>(MotionMessageId::testDumpData), 8, data);
    ++dumpIndex;
}

#endif // MOTION_TESTBENCH
