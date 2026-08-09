#include "BenchDebug.h"
#if BENCHDEBUG
#include <string.h>

const int kLedPin = 13;

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

bool BenchDebug::handleRudderInput(const char* command)
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
    } else if (strcmp(command, "s") == 0) {
        printState();
        return true;
    } else if (strcmp(command, "?") == 0) {
        Serial.println(F("Rudder Commands:"));
        Serial.println(F("r-: calibrate rudder min (hold full LEFT)"));
        Serial.println(F("r0: calibrate rudder center (release pedals)"));
        Serial.println(F("r+: calibrate rudder max (hold full RIGHT)"));
        Serial.println(F("l0/l1: calibrate LEFT brake released/pressed"));
        Serial.println(F("b0/b1: calibrate RIGHT brake released/pressed"));
        Serial.println(F("s: show current state"));
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

    RudderStateUpdate update = rudder->getStateUpdate();
    // Gate the auto-print, not getStateUpdate() itself: at the tightened
    // change threshold a moving pedal trips `changed` almost every pass, and
    // Serial.print() blocks once the ~70-char line overruns the TX ring —
    // which would throttle loop() and stretch the filter's effective time
    // constant well past its design value. The manual 's' command stays
    // unthrottled since it is operator-paced, not loop-paced.
    if (update.changed && millis() - lastPrintMs >= 100) {
        lastPrintMs = millis();
        printState();
    }
}
#endif
