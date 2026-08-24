#ifndef TESTBENCH_H
#define TESTBENCH_H
#include <Arduino.h>
#include "Configuration.h"

#if MOTION_TESTBENCH

#include "MotionActor.h"

class BaseCAN;

// Smoothness test bench (env:nanoatmega328new_testbench only).
//
// The actor's hardware UART is occupied by the Kangaroo, so tests are triggered
// over CAN from the gateway's bench console (actorTestStart/Abort/DumpRequest)
// and results are sampled into RAM, then dumped over CAN (testDumpHeader/Data/
// Status) once the run is finished - never live, so measuring doesn't perturb
// the measured Kangaroo link.
//
// actorTestStart payload:
//   [0]=nodeId [1]=strategy [2]=rateHz [3]=amplitudePct [4]=channelMask
//   [5]=sampleMode [6]=durationSec [7]=param
//
// strategy: 0=single long move (homing-style baseline)
//           1=step stream, current production speed algo (floor range/5, +20%)
//           2=step stream, exact speed (delta/dt, no floor, no headroom)
//           3=strategy 1 with Kangaroo streaming(true) (no reply round-trips)
//           4=strategy 2 with Kangaroo streaming(true)
//           9=live passthrough: don't generate, instrument the production 0x110
//             demand path instead (arrival cadence / cmd duration / sparse getP)
// sampleMode: 0=none, 1=getP position series (strategy 0 only, ~50 Hz),
//             2=timing series (one sample per cycle: command duration),
//             3=getP position series interleaved every param-th cycle
class TestBench
{
public:
    TestBench(MotionActor *motionActor, BaseCAN *canBus);

    // CAN entry points, called from CAN::handleFrame (payload already nodeId-matched)
    void startTest(uint8_t len, const uint8_t *data);
    void abortTest();
    void requestDump();

    void loop();

    bool isRunning() const { return state == State::prepositioning || state == State::running; }
    bool isPassthroughActive() const { return state == State::running && strategy == 9; }

    // Strategy 9 hook: called after each applied production demand.
    void noteDemandApplied(uint32_t cmdDurationUs);

private:
    enum class State : uint8_t
    {
        idle,
        prepositioning, // glide to the window start before the measured run
        running,
        done,
        dumping,
    };

    enum SampleKind : uint8_t
    {
        kSampleNone = 0,
        kSamplePosition = 1, // value = position, 0..65535 across the logical range
        kSampleTiming = 2,   // value = command duration in 10 us units
    };

    struct Sample
    {
        uint16_t tOffsetMs;
        uint16_t value;
    };

    // 160 * 4 bytes = 640 B. The ATmega328 has 2 KB total: 240 samples left only
    // ~380 B of stack after heap and crashed the board - keep generous headroom.
    static constexpr uint16_t kMaxSamples = 160;
    static constexpr uint16_t kPositionSampleIntervalMs = 20; // ~50 Hz for strategy 0
    static constexpr uint32_t kLegDurationMs = 3000;          // window traversal time per leg
    static constexpr uint32_t kDumpFrameIntervalMs = 5;
    static constexpr uint32_t kPrepositionTimeoutMs = 12000;
    static constexpr uint32_t kPrepositionPollIntervalMs = 100;

    void tickPreposition();
    void tickRun();
    void tickDump();
    void finishRun(uint8_t resultState);
    void issueStepCommands(int32_t target);
    void samplePositionIfDue(bool force);
    void recordSample(uint16_t value);
    bool readPosition(uint8_t channel, int32_t &position);
    uint16_t normalizePosition(uint8_t channel, int32_t position) const;
    int32_t windowMin(uint8_t channel) const;
    int32_t windowMax(uint8_t channel) const;
    void setStreaming(bool enabled);
    void sendStatus(uint8_t resultState);
    void sendDumpHeader();

    MotionActor *motionActor;
    BaseCAN *canBus;

    State state = State::idle;

    // Test parameters (from actorTestStart)
    uint8_t strategy = 0;
    uint8_t rateHz = 60;
    uint8_t amplitudePct = 30;
    uint8_t channelMask = 0x01;
    uint8_t sampleMode = kSampleNone;
    uint8_t durationSec = 10;
    uint8_t param = 1;

    // Run state
    uint32_t runStartMs = 0;
    uint32_t periodUs = 16667;
    uint32_t nextTickUs = 0;
    uint32_t nextPositionSampleMs = 0;
    uint32_t lastTickMs = 0;      // for strategy 1/3 dt measurement (production algo)
    int32_t lastTarget[2] = {0, 0};
    bool haveLastTarget = false;
    bool movingUp = true;
    int32_t legStartTarget[2] = {0, 0};
    uint32_t legStartMs = 0;
    uint8_t firstActiveChannel = 0;

    // Preposition state
    uint32_t prepositionStartMs = 0;
    uint32_t nextPrepositionPollMs = 0;

    // Aggregate stats (reported via testStatus)
    uint16_t cyclesDone = 0;
    uint16_t missedCycles = 0;
    uint16_t maxCmdDurationUs10 = 0;
    uint8_t resultState = 0; // 0 running, 1 done, 2 aborted, 3 error

    // Sample buffer. Static so it shows up in the build's RAM accounting (a
    // heap-allocated buffer hides from the size check and can silently collide
    // with the stack on the ATmega328).
    static Sample samples[kMaxSamples];
    uint16_t sampleCount = 0;
    uint8_t sampleKind = kSampleNone;

    // Dump state
    uint16_t dumpIndex = 0;
    uint32_t nextDumpFrameMs = 0;
};

#endif // MOTION_TESTBENCH
#endif // TESTBENCH_H
