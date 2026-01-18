#include <BenchDebug.h>

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
    uint32_t leftEnc = static_cast<uint32_t>(leftTankLevelKg * 100.);
    data[3] = static_cast<uint8_t>(leftEnc & 0xff);
    data[2] = static_cast<uint8_t>((leftEnc >> 8) & 0xff);
    data[1] = static_cast<uint8_t>((leftEnc >> 16) & 0xff);
    data[0]= static_cast<uint8_t>((leftEnc >> 24) & 0xff);

    uint32_t rightEnc = static_cast<uint32_t>(rightTankLevelKg * 100.);
    data[7] = static_cast<uint8_t>(rightEnc & 0xff);
    data[6] = static_cast<uint8_t>((rightEnc >> 8) & 0xff);
    data[5] = static_cast<uint8_t>((rightEnc >> 16) & 0xff);
    data[4]= static_cast<uint8_t>((rightEnc >> 24) & 0xff);

    Serial.print("Send FuelLevel with Data: ");
    char msgString[128]; // Array to store serial string
    for (byte i = 0; i < 8; i++) {
      sprintf(msgString, " 0x%.2X", data[i]);
      Serial.print(msgString);
    }
    Serial.println();

    canBus->sendMessage(CanStateId::fuelLevel, 8, data);
}

void BenchDebug::sendCockpitLightLevel() {
    byte data[8] = {0};
    data[0]= cockpitLightLevel;
    canBus->sendMessage(CanStateId::dashboardLight, 1, data);
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
        Serial.println(F("rt<lg>: display fuel level right tank"));
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
            // Instead, we use a fixed size array of strings, which is large enough to hold all possible commands.
            // The maximum number of commands is 10, which is more than enough for this application.
            String commands[kMaxCommandLength];
            int commandCount = 0;
            int lastCommandEnd = 0;
            for (unsigned int i = 0; i < inputBuffer.length(); i++) {
                if (inputBuffer[i] == ' ') {
                    commands[commandCount] = inputBuffer.substring(lastCommandEnd, i);
                    commandCount++;
                    if (commandCount >= kMaxCommandLength) {
                        Serial.println("Too many commands in one line. Maximum is 10.");
                        break;
                    }
                    lastCommandEnd = i + 1;
                }
            }
            commands[commandCount] = inputBuffer.substring(lastCommandEnd);
            commandCount++;

            bool commandExecuted = false;


            // Execute all commands
            for (int i = 0; i < commandCount; i++) {
                commandExecuted = handleAltimeterInput(commands[i]);
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