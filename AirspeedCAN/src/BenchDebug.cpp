#include <BenchDebug.h>
#if BENCHDEBUG
#include <string.h>
#include <stdlib.h>
#include "DebugLog.h"

const int kLedPin = 13;

BenchDebug::BenchDebug(AirspeedIndicator* airspeedIndicator)
: airspeedIndicator(airspeedIndicator)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Airspeed BenchDebug"));

    inputBuffer[0] = '\0';
    inputLength = 0;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    airspeedIndicator->moveNeedle(0.);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

void BenchDebug::printCalibration() {
    const CalibrationTable& table = airspeedIndicator->calibration();
    Serial.print(F("Calibration points ("));
    Serial.print(table.count);
    Serial.print(F("/"));
    Serial.print(kMaxCalibrationPoints);
    Serial.println(F("):"));
    for (uint8_t i = 0; i < table.count; i++) {
        Serial.print(F("  "));
        Serial.print(table.points[i].knots);
        Serial.print(F("kt -> step "));
        Serial.println(table.points[i].step);
    }
}

bool BenchDebug::handleAirspeedInput(char* command) {
    if (continuousTestActive &&
        (strncmp(command, "as", 2) == 0 ||
         strncmp(command, "cl", 2) == 0 ||
         strncmp(command, "br", 2) == 0 ||
         strncmp(command, "ho", 2) == 0 ||
         strncmp(command, "cp", 2) == 0 ||
         strncmp(command, "cw", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }

    if (strncmp(command, "as", 2) == 0) {
        airspeedIndicator->moveNeedle(atof(command + 2));
        return true;
    } else if (strncmp(command, "cl", 2) == 0) {
        airspeedIndicator->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "ho", 2) == 0) {
        Serial.println(F("Homing needle..."));
        airspeedIndicator->home();
        Serial.println(F("Needle homed."));
        return true;
    } else if (strncmp(command, "cp", 2) == 0) {
        // The needle must already sit on the dial mark, put there with cl<degree>.
        const uint16_t knots = static_cast<uint16_t>(atoi(command + 2));
        if (!airspeedIndicator->calibratePoint(knots)) {
            Serial.println(F("Calibration table full. Wipe with 'cw' or reuse a point."));
            return true;
        }
        printCalibration();
        return true;
    } else if (strncmp(command, "cw", 2) == 0) {
        airspeedIndicator->wipeCalibration();
        Serial.println(F("Calibration wiped."));
        printCalibration();
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        airspeedIndicator->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (strncmp(command, "co", 2) == 0) {
        continuousTestActive = true;

        iasMoveFrom = 0;
        iasMoveTo = 0;
        iasMoveStartMillis = millis();
        iasMoveDurationMillis = 0; // forces an immediate target pick on the first tick
        lastIASDeliveryMillis = millis();
        return true;
    } else if (strncmp(command, "cx", 2) == 0) {
        continuousTestActive = false;
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("Airspeed Indicator Commands:"));
        Serial.println(F("as<value>: display given airspeed in knots"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("ho: home the needle (start of every calibration)"));
        Serial.println(F("cl<degree>: move needle to a raw dial angle"));
        Serial.println(F("cp<knots>: store current needle position as the mark for <knots>"));
        Serial.println(F("cw: wipe all calibration points (factory reset)"));
        Serial.println(F("co: start continuous needle test"));
        Serial.println(F("cx: stop continuous test"));
        Serial.println(F("Calibration: ho, then cl<degree> until the needle sits on a mark,"));
        Serial.println(F("then cp<knots> for that mark. Repeat for every mark you want."));
        printCalibration();
        return true;
    }
    return false;
}

void BenchDebug::handleUserInput()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read();
        if (receivedChar == '\n')
        {
            Serial.println();
            inputBuffer[inputLength] = '\0';

            bool commandExecuted = false;
            char* token = strtok(inputBuffer, " ");
            while (token != nullptr) {
                if (strlen(token) > 0 && handleAirspeedInput(token)) {
                    commandExecuted = true;
                }
                token = strtok(nullptr, " ");
            }

            if (!commandExecuted) {
                Serial.println(F("Unknown command. Type '?' for help."));
            }

            inputLength = 0;
        }
        else if (receivedChar != '\r')
        {
            if (inputLength < kInputBufferSize - 1) {
                inputBuffer[inputLength++] = receivedChar;
                Serial.print(receivedChar);
            }
        }
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

    if (continuousTestActive)
    {
        // Needle: random targets, but interpolated and delivered at 50Hz via
        // the real moveNeedle() API, just like actual CAN telemetry would
        // drive it. Targets span the calibrated range plus a little margin on
        // each side, so the lower/upper clamp gets exercised too.
        if (millis() - lastIASDeliveryMillis >= kIASDeliveryIntervalMs)
        {
            lastIASDeliveryMillis += kIASDeliveryIntervalMs;

            uint32_t elapsed = millis() - iasMoveStartMillis;
            if (elapsed >= iasMoveDurationMillis)
            {
                iasMoveFrom = iasMoveTo;
                const long maxKnots = (long)calibrationMaxKnots(airspeedIndicator->calibration());
                iasMoveTo = random(-10L, maxKnots + 10L);
                iasMoveDurationMillis = random(500, 2000);
                iasMoveStartMillis = millis();
                elapsed = 0;
            }

            float progress = static_cast<float>(elapsed) / static_cast<float>(iasMoveDurationMillis);
            float interpolatedIAS = iasMoveFrom + (iasMoveTo - iasMoveFrom) * progress;
            airspeedIndicator->moveNeedle(interpolatedIAS);
        }
    }

    handleUserInput();
}
#endif
