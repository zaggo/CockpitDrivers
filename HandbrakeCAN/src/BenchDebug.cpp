#include <BenchDebug.h>
#if BENCHDEBUG
const int kLedPin = 13;

BenchDebug::BenchDebug(Handbrake* handbrake): handbrake(handbrake)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Handbrake BenchDebug"));

    inputBuffer = "";

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    position = handbrake->getHandbrakePosition();
    Serial.println(String(F("System running, initial handbrake raw position: ")) + String(position));
}

BenchDebug::~BenchDebug()
{
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleHandbrakeInput(String command) {
    if (command.startsWith("mi")) {
        Serial.println(F("Sampling min position (release handbrake)..."));
        handbrake->calibrateMin();
        Serial.println(F("Min position calibrated."));
        return true;
    }  else if (command.startsWith("ma")) {
        Serial.println(F("Sampling max position (pull handbrake fully)..."));
        handbrake->calibrateMax();
        Serial.println(F("Max position calibrated."));
        return true;
    }  else if (command.startsWith("?")) {
        Serial.println(F("Handbrake Commands:"));
        Serial.println(F("mi: calibrate minimum position (release handbrake first)"));
        Serial.println(F("ma: calibrate maximum position (pull handbrake fully)"));
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
                commandExecuted = handleHandbrakeInput(commands[i]);
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

    uint8_t newPosition = handbrake->getHandbrakePosition();
    if (abs((int)newPosition - (int)position) > 3) {
        position = newPosition;
        Serial.println(String(F("Handbrake position: ")) + String(position) + String(F(" (raw: ")) + String(handbrake->getRawPosition()) + String(F(")")));
    }
}
#endif