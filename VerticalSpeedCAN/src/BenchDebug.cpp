#include <BenchDebug.h>
#if BENCHDEBUG
#include <string.h>
#include <stdlib.h>
#include "DebugLog.h"

const int kLedPin = 13;

BenchDebug::BenchDebug(VerticalSpeedIndicator* verticalSpeedIndicator)
: verticalSpeedIndicator(verticalSpeedIndicator)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Vertical Speed BenchDebug"));

    inputBuffer[0] = '\0';
    inputLength = 0;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    verticalSpeedIndicator->moveNeedle(0.);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

void BenchDebug::printCalibration() {
    const CalibrationTable& table = verticalSpeedIndicator->calibration();
    Serial.print(F("Calibration points ("));
    Serial.print(table.count);
    Serial.print(F("/"));
    Serial.print(kMaxCalibrationPoints);
    Serial.println(F("):"));
    if (table.count == 0) {
        Serial.println(F("  none — needle parks on the home stop"));
    }
    for (uint8_t i = 0; i < table.count; i++) {
        Serial.print(F("  "));
        Serial.print(table.points[i].fpm);
        Serial.print(F("fpm -> step "));
        Serial.println(table.points[i].step);
    }
}

bool BenchDebug::handleVerticalSpeedInput(char* command) {
    if (continuousTestActive &&
        (strncmp(command, "vs", 2) == 0 ||
         strncmp(command, "cl", 2) == 0 ||
         strncmp(command, "br", 2) == 0 ||
         strncmp(command, "ho", 2) == 0 ||
         strncmp(command, "cp", 2) == 0 ||
         strncmp(command, "cw", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }

    if (strncmp(command, "vs", 2) == 0) {
        verticalSpeedIndicator->moveNeedle(atof(command + 2));
        return true;
    } else if (strncmp(command, "cl", 2) == 0) {
        verticalSpeedIndicator->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "ho", 2) == 0) {
        Serial.println(F("Homing needle..."));
        verticalSpeedIndicator->home();
        Serial.println(F("Needle homed."));
        return true;
    } else if (strncmp(command, "cp", 2) == 0) {
        // The needle must already sit on the dial mark, put there with cl<degree>.
        // Negative values are expected here — cp-500 teaches the 500fpm descent
        // mark, and cp0 teaches the level-flight mark, which is nowhere near
        // the mechanical home stop.
        const int16_t fpm = static_cast<int16_t>(atoi(command + 2));
        if (!verticalSpeedIndicator->calibratePoint(fpm)) {
            Serial.println(F("Calibration table full. Wipe with 'cw' or reuse a point."));
            return true;
        }
        printCalibration();
        return true;
    } else if (strncmp(command, "cw", 2) == 0) {
        verticalSpeedIndicator->wipeCalibration();
        Serial.println(F("Calibration wiped."));
        printCalibration();
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        verticalSpeedIndicator->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (strncmp(command, "co", 2) == 0) {
        continuousTestActive = true;

        vsiMoveFrom = 0;
        vsiMoveTo = 0;
        vsiMoveStartMillis = millis();
        vsiMoveDurationMillis = 0; // forces an immediate target pick on the first tick
        lastVSIDeliveryMillis = millis();
        return true;
    } else if (strncmp(command, "cx", 2) == 0) {
        continuousTestActive = false;
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("Vertical Speed Indicator Commands:"));
        Serial.println(F("vs<value>: display given climb rate in ft/min (negative = descent)"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("ho: home the needle (start of every calibration)"));
        Serial.println(F("cl<degree>: move needle to a raw dial angle"));
        Serial.println(F("cp<fpm>: store current needle position as the mark for <fpm>"));
        Serial.println(F("cw: wipe all calibration points (factory reset)"));
        Serial.println(F("co: start continuous needle test"));
        Serial.println(F("cx: stop continuous test"));
        Serial.println(F("Calibration: ho, then cl<degree> until the needle sits on a mark,"));
        Serial.println(F("then cp<fpm> for that mark. Repeat for every mark you want —"));
        Serial.println(F("including cp0, since 0fpm is not the home stop."));
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
                if (strlen(token) > 0 && handleVerticalSpeedInput(token)) {
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
        // each side, so both clamps get exercised too.
        if (millis() - lastVSIDeliveryMillis >= kVSIDeliveryIntervalMs)
        {
            lastVSIDeliveryMillis += kVSIDeliveryIntervalMs;

            uint32_t elapsed = millis() - vsiMoveStartMillis;
            if (elapsed >= vsiMoveDurationMillis)
            {
                vsiMoveFrom = vsiMoveTo;
                const CalibrationTable& table = verticalSpeedIndicator->calibration();
                const long minFpm = (long)calibrationMinFpm(table);
                const long maxFpm = (long)calibrationMaxFpm(table);
                vsiMoveTo = random(minFpm - 100L, maxFpm + 100L);
                vsiMoveDurationMillis = random(500, 2000);
                vsiMoveStartMillis = millis();
                elapsed = 0;
            }

            float progress = static_cast<float>(elapsed) / static_cast<float>(vsiMoveDurationMillis);
            float interpolatedVSI = vsiMoveFrom + (vsiMoveTo - vsiMoveFrom) * progress;
            verticalSpeedIndicator->moveNeedle(interpolatedVSI);
        }
    }

    handleUserInput();
}
#endif
