#include "BenchDebug.h"
#if BENCHDEBUG
#include <string.h>

const int kLedPin = 13;

namespace {

struct AdcStats {
    uint16_t minValue;
    uint16_t maxValue;
    uint32_t sum;
    uint16_t count;
};

void statsInit(AdcStats& stats)
{
    stats.minValue = 1023;
    stats.maxValue = 0;
    stats.sum      = 0;
    stats.count    = 0;
}

void statsAdd(AdcStats& stats, uint16_t value)
{
    if (value < stats.minValue) {
        stats.minValue = value;
    }
    if (value > stats.maxValue) {
        stats.maxValue = value;
    }
    stats.sum += value;
    stats.count++;
}

// Distance between two calibration points, direction-agnostic: inverted sensor
// wiring puts the larger raw value at either end.
uint16_t spanOf(uint16_t a, uint16_t b)
{
    return (a > b) ? (uint16_t)(a - b) : (uint16_t)(b - a);
}

void statsPrint(const __FlashStringHelper* label, const AdcStats& stats)
{
    Serial.print(label);
    Serial.print(F(" min "));
    Serial.print(stats.minValue);
    Serial.print(F(" max "));
    Serial.print(stats.maxValue);
    Serial.print(F(" spread "));
    Serial.print((uint16_t)(stats.maxValue - stats.minValue));
    Serial.print(F(" avg "));
    Serial.println(stats.count ? (uint16_t)((stats.sum + stats.count / 2) / stats.count) : 0);
}

}

BenchDebug::BenchDebug(Rudder* rudder): rudder(rudder)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Rudder BenchDebug — type '?' for help"));

    inputBuffer[0] = '\0';
    inputLength = 0;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    printState();
}

BenchDebug::~BenchDebug()
{
}

void BenchDebug::printState()
{
    RudderState state = rudder->getState();
    Serial.print(F("rud "));
    Serial.print(state.rudder);
    Serial.print(F(" (raw "));
    Serial.print(rudder->getRawRudder());
    Serial.print(F(" q6 "));
    Serial.print(rudder->getFilteredRudderQ6());
    Serial.print(F(")  lBrk "));
    Serial.print(state.leftBrake);
    Serial.print(F(" (raw "));
    Serial.print(rudder->getRawLeftBrake());
    Serial.print(F(" q6 "));
    Serial.print(rudder->getFilteredLeftBrakeQ6());
    Serial.print(F(")  rBrk "));
    Serial.print(state.rightBrake);
    Serial.print(F(" (raw "));
    Serial.print(rudder->getRawRightBrake());
    Serial.print(F(" q6 "));
    Serial.print(rudder->getFilteredRightBrakeQ6());
    Serial.println(')');
}

// Diagnostic: characterises the raw ADC noise on each axis. printState()'s raw
// column is a single extra analogRead at print time, so it never shows what the
// filters are actually being fed between prints — this does.
//
// Three passes, so the result tells apart the two candidate noise sources:
//   A  same channel order and cadence as Rudder::sample() — the filters' real diet
//   B  one channel, no multiplexer switching at all
//   C  round robin, but the brake channel is read twice and only the second read
//      kept, the standard workaround for a sample-and-hold that has not settled
//      on a high-impedance source
// A noisy but B quiet  => multiplexer/source-impedance, not the pot.
// A and B both noisy   => the analog signal itself (pot, wiring, contact).
// C quiet while A is not => confirms the settling explanation directly.
void BenchDebug::noiseProbe()
{
    const uint16_t kProbeSamples = 200;

    AdcStats rudderRoundRobin, leftRoundRobin, rightRoundRobin;
    statsInit(rudderRoundRobin);
    statsInit(leftRoundRobin);
    statsInit(rightRoundRobin);
    for (uint16_t i = 0; i < kProbeSamples; i++) {
        statsAdd(rudderRoundRobin, analogRead(kRudderPin));
        statsAdd(leftRoundRobin,   analogRead(kLeftBrakePin));
        statsAdd(rightRoundRobin,  analogRead(kRightBrakePin));
        delay(2);
    }

    AdcStats leftSingleChannel;
    statsInit(leftSingleChannel);
    for (uint16_t i = 0; i < kProbeSamples; i++) {
        statsAdd(leftSingleChannel, analogRead(kLeftBrakePin));
        delay(2);
    }

    AdcStats leftDoubleRead;
    statsInit(leftDoubleRead);
    for (uint16_t i = 0; i < kProbeSamples; i++) {
        analogRead(kRudderPin);
        analogRead(kLeftBrakePin);
        statsAdd(leftDoubleRead, analogRead(kLeftBrakePin));
        analogRead(kRightBrakePin);
        delay(2);
    }

    Serial.println(F("ADC noise probe (200 samples each, hold pedals still):"));
    statsPrint(F("A rud       "), rudderRoundRobin);
    statsPrint(F("A lBrk      "), leftRoundRobin);
    statsPrint(F("A rBrk      "), rightRoundRobin);
    statsPrint(F("B lBrk solo "), leftSingleChannel);
    statsPrint(F("C lBrk x2   "), leftDoubleRead);
}

// Every calibration point is only as good as the travel it spans: a small span
// means each raw ADC count is worth many wire units, so noise that would be
// invisible on a wide axis swings the reported value hard. Printing the span
// right after sampling a point makes a bad mechanical/electrical range obvious
// at the moment it is calibrated, instead of at fly time.
void BenchDebug::printCalibration()
{
    const RudderConfig& config = rudder->getConfig();

    Serial.print(F("cal rud "));
    Serial.print(config.rudderMin);
    Serial.print('/');
    Serial.print(config.rudderCenter);
    Serial.print('/');
    Serial.print(config.rudderMax);
    Serial.print(F(" travel "));
    Serial.print(spanOf(config.rudderMin, config.rudderMax));
    Serial.print(F(" (L "));
    Serial.print(spanOf(config.rudderCenter, config.rudderMin));
    Serial.print(F(" R "));
    Serial.print(spanOf(config.rudderCenter, config.rudderMax));
    Serial.println(')');

    Serial.print(F("cal lBrk "));
    Serial.print(config.leftBrakeMin);
    Serial.print('/');
    Serial.print(config.leftBrakeMax);
    Serial.print(F(" travel "));
    Serial.println(spanOf(config.leftBrakeMin, config.leftBrakeMax));

    Serial.print(F("cal rBrk "));
    Serial.print(config.rightBrakeMin);
    Serial.print('/');
    Serial.print(config.rightBrakeMax);
    Serial.print(F(" travel "));
    Serial.println(spanOf(config.rightBrakeMin, config.rightBrakeMax));
}

bool BenchDebug::handleCalibrationInput(const char* command)
{
    if (strcmp(command, "r-") == 0) {
        Serial.println(F("Sampling rudder MIN (hold full LEFT)..."));
        rudder->calibrateRudderMin();
        Serial.println(F("Rudder min calibrated."));
        return true;
    } else if (strcmp(command, "r0") == 0) {
        Serial.println(F("Sampling rudder CENTER (release pedals)..."));
        rudder->calibrateRudderCenter();
        Serial.println(F("Rudder center calibrated."));
        return true;
    } else if (strcmp(command, "r+") == 0) {
        Serial.println(F("Sampling rudder MAX (hold full RIGHT)..."));
        rudder->calibrateRudderMax();
        Serial.println(F("Rudder max calibrated."));
        return true;
    } else if (strcmp(command, "l0") == 0) {
        Serial.println(F("Sampling LEFT brake released..."));
        rudder->calibrateLeftBrakeMin();
        Serial.println(F("Left brake min calibrated."));
        return true;
    } else if (strcmp(command, "l1") == 0) {
        Serial.println(F("Sampling LEFT brake fully pressed..."));
        rudder->calibrateLeftBrakeMax();
        Serial.println(F("Left brake max calibrated."));
        return true;
    } else if (strcmp(command, "b0") == 0) {
        Serial.println(F("Sampling RIGHT brake released..."));
        rudder->calibrateRightBrakeMin();
        Serial.println(F("Right brake min calibrated."));
        return true;
    } else if (strcmp(command, "b1") == 0) {
        Serial.println(F("Sampling RIGHT brake fully pressed..."));
        rudder->calibrateRightBrakeMax();
        Serial.println(F("Right brake max calibrated."));
        return true;
    }
    return false;
}

bool BenchDebug::handleRudderInput(const char* command)
{
    if (handleCalibrationInput(command)) {
        printCalibration();
        return true;
    }

    if (strcmp(command, "s") == 0) {
        printState();
        return true;
    } else if (strcmp(command, "n") == 0) {
        noiseProbe();
        return true;
    } else if (strcmp(command, "?") == 0) {
        Serial.println(F("Rudder Commands:"));
        Serial.println(F("r-: calibrate rudder min (hold full LEFT)"));
        Serial.println(F("r0: calibrate rudder center (release pedals)"));
        Serial.println(F("r+: calibrate rudder max (hold full RIGHT)"));
        Serial.println(F("l0/l1: calibrate LEFT brake released/pressed"));
        Serial.println(F("b0/b1: calibrate RIGHT brake released/pressed"));
        Serial.println(F("s: show current state"));
        Serial.println(F("n: ADC noise probe (takes ~1.5s, hold pedals still)"));
        return true;
    }
    return false;
}

void BenchDebug::handleUserInput()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read();

        if (receivedChar == '\r') {
            continue;
        }

        if (receivedChar != '\n')
        {
            if (inputLength < kInputBufferSize - 1) {
                inputBuffer[inputLength++] = receivedChar;
                inputBuffer[inputLength] = '\0';
                Serial.print(receivedChar);
            }
            continue;
        }

        Serial.println();

        // Split on spaces in place and run each token as its own command.
        bool commandExecuted = false;
        char* cursor = inputBuffer;
        while (*cursor != '\0')
        {
            while (*cursor == ' ') {
                cursor++;
            }
            if (*cursor == '\0') {
                break;
            }

            char* tokenStart = cursor;
            while (*cursor != '\0' && *cursor != ' ') {
                cursor++;
            }
            if (*cursor == ' ') {
                *cursor = '\0';
                cursor++;
            }

            if (handleRudderInput(tokenStart)) {
                commandExecuted = true;
            } else {
                Serial.print(F("Unknown command: "));
                Serial.println(tokenStart);
            }
        }

        if (!commandExecuted) {
            Serial.println(F("Type '?' for help."));
        }

        inputBuffer[0] = '\0';
        inputLength = 0;
    }
}

void BenchDebug::loop()
{
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    rudder->sample();

    handleUserInput();

    // Gate BEFORE consuming the update, not after: getStateUpdate() latches
    // _lastReportedState whenever it reports changed == true, regardless of
    // whether we go on to print. Calling it on every pass while only gating
    // the print would advance the latch on gate-closed passes too, silently
    // and permanently dropping any change that lands while the gate is
    // closed. Checking the time gate first means the latch only advances on
    // passes that will actually print, so an unprinted change leaves
    // _lastReportedState stale and gets picked up as an accumulated delta on
    // the next gate-open pass.
    if (millis() - lastPrintMs >= kPrintIntervalMs) {
        RudderStateUpdate update = rudder->getStateUpdate();
        if (update.changed) {
            lastPrintMs = millis();
            printState();
        }
    }
}
#endif
