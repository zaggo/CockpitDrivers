#include <BenchDebug.h>
#if BENCHDEBUG
const int kBenchLedPin = 13;

BenchDebug::BenchDebug(Starter* starter): starter(starter)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Starter BenchDebug"));

    inputBuffer = "";

    pinMode(kBenchLedPin, OUTPUT);
    digitalWrite(kBenchLedPin, heartbeatLedOn);

    Serial.println(String(F("System running, initial switch position: ")) + String(starter->getPosition()));
}

BenchDebug::~BenchDebug()
{
}

bool BenchDebug::handleStarterInput(String command) {
    if (command.startsWith("l0")) {
        starter->setLed(false);
        Serial.println(F("Starter LED off"));
        return true;
    } else if (command.startsWith("l1")) {
        starter->setLed(true);
        Serial.println(F("Starter LED on"));
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("Starter Commands:"));
        Serial.println(F("l1: starter LED on"));
        Serial.println(F("l0: starter LED off"));
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
            inputBuffer.trim();

            bool commandExecuted = handleStarterInput(inputBuffer);

            if (!commandExecuted) {
                Serial.println(F("Unknown command. Type '?' for help."));
            }

            inputBuffer = "";
        }
        else
        {
            inputBuffer += receivedChar;
            Serial.print(receivedChar);
        }
    }
}

void BenchDebug::loop()
{
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kBenchLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    handleUserInput();

    StarterPositionUpdate update = starter->getPositionUpdate();
    if (update.changed) {
        Serial.println(String(F("Starter position: ")) + String(update.position));
    }
}
#endif
