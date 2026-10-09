#include <BenchDebug.h>
#if BENCHDEBUG
#include <string.h>
#include <stdlib.h>
#include "DebugLog.h"

const int kLedPin = 13;

BenchDebug::BenchDebug(TurnCoordinator* turnCoordinator)
: turnCoordinator(turnCoordinator)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Turn Coordinator BenchDebug"));

    inputBuffer[0] = '\0';
    inputLength = 0;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    turnCoordinator->setTurnCoordinator(0., 0.);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

void BenchDebug::printTable(const __FlashStringHelper* name, const CalibrationTable& table) {
    Serial.print(name);
    Serial.print(F(" calibration points ("));
    Serial.print(table.count);
    Serial.print(F("/"));
    Serial.print(kMaxCalibrationPoints);
    Serial.println(F("):"));
    if (table.count == 0) {
        Serial.println(F("  none - needle parks on the home stop"));
    }
    for (uint8_t i = 0; i < table.count; i++) {
        Serial.print(F("  "));
        Serial.print(table.points[i].value);
        Serial.print(F("deg -> step "));
        Serial.println(table.points[i].step);
    }
}

void BenchDebug::printCalibration() {
    printTable(F("Turn rate"), turnCoordinator->calibration(turnRate));
    printTable(F("Slip"), turnCoordinator->calibration(sideSlip));
}

bool BenchDebug::handleTurnCoordinatorInput(char* command) {
    if (continuousTestActive &&
        (strncmp(command, "tr", 2) == 0 ||
         strncmp(command, "sl", 2) == 0 ||
         strncmp(command, "ct", 2) == 0 ||
         strncmp(command, "cs", 2) == 0 ||
         strncmp(command, "pt", 2) == 0 ||
         strncmp(command, "ps", 2) == 0 ||
         strncmp(command, "ho", 2) == 0 ||
         strncmp(command, "cw", 2) == 0)) {
        Serial.println(F("Continuous test running. Type 'cx' to stop."));
        return true;
    }

    if (strncmp(command, "tr", 2) == 0) {
        turnCoordinator->moveNeedle(turnRate, atof(command + 2));
        return true;
    } else if (strncmp(command, "sl", 2) == 0) {
        turnCoordinator->moveNeedle(sideSlip, atof(command + 2));
        return true;
    } else if (strncmp(command, "ct", 2) == 0) {
        turnCoordinator->moveNeedle(turnRate, atof(command + 2), true);
        return true;
    } else if (strncmp(command, "cs", 2) == 0) {
        turnCoordinator->moveNeedle(sideSlip, atof(command + 2), true);
        return true;
    } else if (strncmp(command, "ho", 2) == 0) {
        Serial.println(F("Homing both needles..."));
        turnCoordinator->home();
        Serial.println(F("Needles homed."));
        return true;
    } else if (strncmp(command, "pt", 2) == 0 || strncmp(command, "ps", 2) == 0) {
        // The needle must already sit on the dial mark, put there with ct/cs.
        // Negative values are expected here - pt-90 teaches the left mark and
        // pt0 the level mark, which is nowhere near the mechanical home stop.
        const Needle needle = (command[1] == 't') ? turnRate : sideSlip;
        const int16_t deg = static_cast<int16_t>(atoi(command + 2));
        // "ct90 pt90" on one line runs both before loop() has stepped once, and
        // a fast-typed pt lands mid-travel - either would store the pre-move
        // step for the mark. Refuse instead of recording a wrong point.
        if (!turnCoordinator->isSettled(needle)) {
            Serial.println(F("Needle still moving - wait until it stops, then repeat pt/ps."));
            return true;
        }
        if (!turnCoordinator->calibratePoint(needle, deg)) {
            Serial.println(F("Calibration table full. Wipe with 'cw' or reuse a point."));
            return true;
        }
        printCalibration();
        return true;
    } else if (strncmp(command, "cw", 2) == 0) {
        turnCoordinator->wipeCalibration();
        Serial.println(F("Calibration wiped (both needles)."));
        printCalibration();
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        turnCoordinator->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (strncmp(command, "co", 2) == 0) {
        continuousTestActive = true;
        for (uint8_t i = 0; i < needleCount; i++) {
            moves[i].from = 0;
            moves[i].to = 0;
            moves[i].startMillis = millis();
            moves[i].durationMillis = 0; // forces an immediate target pick on the first tick
        }
        lastDeliveryMillis = millis();
        return true;
    } else if (strncmp(command, "cx", 2) == 0) {
        continuousTestActive = false;
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("Turn Coordinator Commands:"));
        Serial.println(F("tr<deg>: turn rate needle via calibration (negative = left)"));
        Serial.println(F("sl<deg>: slip ball via calibration (negative = left)"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("ho: home both needles (start of every calibration)"));
        Serial.println(F("ct<degree> / cs<degree>: move turn rate / slip needle to a raw dial angle"));
        Serial.println(F("pt<deg> / ps<deg>: store current turn rate / slip position as the mark for <deg>"));
        Serial.println(F("cw: wipe all calibration points of both needles (factory reset)"));
        Serial.println(F("co: start continuous test on both needles"));
        Serial.println(F("cx: stop continuous test"));
        Serial.println(F("Calibration: ho, then ct<degree> until the needle sits on a mark,"));
        Serial.println(F("then pt<deg> for that mark. Repeat for every mark you want -"));
        Serial.println(F("including pt0, since 0 is not the home stop. Same with cs/ps for the ball."));
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
                if (strlen(token) > 0 && handleTurnCoordinatorInput(token)) {
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
        // Both needles: random targets, interpolated and delivered at 50Hz via
        // the real moveNeedle() API, just like actual CAN telemetry would drive
        // them. Targets span the calibrated range plus a little margin on each
        // side, so both clamps get exercised too.
        if (millis() - lastDeliveryMillis >= kDeliveryIntervalMs)
        {
            lastDeliveryMillis += kDeliveryIntervalMs;

            for (uint8_t i = 0; i < needleCount; i++)
            {
                const Needle needle = static_cast<Needle>(i);
                NeedleMove& move = moves[i];

                uint32_t elapsed = millis() - move.startMillis;
                if (elapsed >= move.durationMillis)
                {
                    move.from = move.to;
                    const CalibrationTable& table = turnCoordinator->calibration(needle);
                    const long minValue = (long)calibrationMinValue(table);
                    const long maxValue = (long)calibrationMaxValue(table);
                    // Margin scaled to the needle: a few degrees past the slip
                    // ball's +-8, a good deal past the turn needle's +-90.
                    const long margin = (needle == sideSlip) ? 2L : 20L;
                    move.to = random(minValue - margin, maxValue + margin + 1);
                    move.durationMillis = random(500, 2000);
                    move.startMillis = millis();
                    elapsed = 0;
                }

                float progress = static_cast<float>(elapsed) / static_cast<float>(move.durationMillis);
                float interpolated = move.from + (move.to - move.from) * progress;
                turnCoordinator->moveNeedle(needle, interpolated);
            }
        }
    }

    handleUserInput();
}
#endif
