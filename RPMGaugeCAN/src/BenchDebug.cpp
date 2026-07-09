#include <BenchDebug.h>
#if BENCHDEBUG
#include <string.h>
#include <stdlib.h>

const int kLedPin = 13;

BenchDebug::BenchDebug(RPMGauge* rpmGauge, Odometer* odometer)
: rpmGauge(rpmGauge)
, odometer(odometer)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("RPMGauge BenchDebug"));

    inputBuffer[0] = '\0';
    inputLength = 0;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    rpmGauge->moveNeedle(0);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

bool BenchDebug::handleRPMGaugeInput(char* command) {
    if (strncmp(command, "rp", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2));
        return true;
    } else if (strncmp(command, "od", 2) == 0) {
        float digits[6];
        odometer->secondsToDigits(atof(command + 2), digits);
        odometer->displayNumber(digits);
        return true;
    } else if (strncmp(command, "cl", 2) == 0) {
        rpmGauge->moveNeedle(atof(command + 2), true);
        return true;
    } else if (strncmp(command, "br", 2) == 0) {
        rpmGauge->setBrightness(static_cast<uint8_t>(atoi(command + 2)));
        return true;
    } else if (command[0] == '?') {
        Serial.println(F("RPM Gauge Commands:"));
        Serial.println(F("rp<value>: display given RPM"));
        Serial.println(F("od<value>: display given seconds as odometer value"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("cl<degree>: calibrate needle"));
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
                if (strlen(token) > 0 && handleRPMGaugeInput(token)) {
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

    handleUserInput();
}
#endif
