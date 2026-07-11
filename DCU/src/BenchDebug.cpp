#include <BenchDebug.h>
#include "WireEncoding.h"
#include "CommandTokenizer.h"

#if BENCHDEBUG
const int kLedPin = 13;

BenchDebug::BenchDebug(CAN* canBus) : canBus(canBus)
{
    Serial.begin(115200);
    Serial.println(F("DCU BenchDebug"));

    inputBuffer = "";

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    Serial.println(F("BenchDebug running!"));
}

BenchDebug::~BenchDebug()
{
}

void BenchDebug::sendFuelLevel() {
    byte data[8] = {0};
    packBE16(data + 0, static_cast<uint16_t>(leftTankLevelKg * 100.));
    packBE16(data + 2, static_cast<uint16_t>(rightTankLevelKg * 100.));

    Serial.print("Send FuelLevel with Data: ");
    char msgString[128]; // Array to store serial string
    for (byte i = 0; i < 8; i++) {
      sprintf(msgString, " 0x%.2X", data[i]);
      Serial.print(msgString);
    }
    Serial.println();

    canBus->sendMessage(CanMessageId::fuelLevel, 8, data);
}

void BenchDebug::sendCockpitLightLevel() {
    byte data[8] = {0};
    uint16_t panelDim1000 = static_cast<uint16_t>(static_cast<float>(cockpitLightLevel) / 255. * 1000.);
    packBE16(data + 0, panelDim1000);
    canBus->sendMessage(CanMessageId::lights, 8, data);
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleAltimeterInput(String command) {
    if (command.startsWith("lt")) {
        String rString = command.substring(2);
        rString.trim();
        leftTankLevelKg = rString.toFloat();
        Serial.println(String(F("Left Tank set kg="))+leftTankLevelKg);
        sendFuelLevel();
        return true;
    } else if (command.startsWith("rt")) {
        String rString = command.substring(2);
        rString.trim();
        rightTankLevelKg = rString.toFloat();
        Serial.println(String(F("Right Tank set kg="))+rightTankLevelKg);
        sendFuelLevel();
        return true;
    } else if (command.startsWith("cl")) {
        String rString = command.substring(2);
        rString.trim();
        cockpitLightLevel = rString.toInt();
        Serial.println(String(F("Cockpit light set brightness="))+cockpitLightLevel);
        sendCockpitLightLevel();
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("DCU Commands:"));
        Serial.println(F("lt<kg>: display fuel level left tank"));
        Serial.println(F("rt<kg>: display fuel level right tank"));
        Serial.println(F("cl<0..255>: set light brightness"));
        return true;
    }       
    return false;
}

void BenchDebug::handleUserInput()
{
    static String inputBuffer = ""; // Zwischenspeicher für serielle Eingaben

    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read(); // Einzelnes Zeichen lesen
        if (receivedChar == '\n')
        {                       // Enter erkannt
            Serial.println();   // Neue Zeile
            inputBuffer.trim(); // Eingabe bereinigen (Leerzeichen etc.)

            // Split the inputBuffer into a vector of single commands. Since this program is executed on an Arduino, we can't use the std::vector class.
            char lineBuffer[64];
            inputBuffer.toCharArray(lineBuffer, sizeof(lineBuffer));

            char* tokens[kMaxCommandLength];
            size_t commandCount = tokenizeCommands(lineBuffer, tokens, kMaxCommandLength);

            bool commandExecuted = false;

            // Execute all commands
            for (size_t i = 0; i < commandCount; i++) {
                commandExecuted = handleAltimeterInput(String(tokens[i])) || commandExecuted;
            }

            if (!commandExecuted) {
                Serial.println(F("Unknown command. Type '?' for help."));
            }

            inputBuffer = ""; // Buffer leeren
        }
        else
        {
            inputBuffer += receivedChar; // Zeichen an den Buffer anhängen
            Serial.print(receivedChar);  // Eingabe zurückgeben
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