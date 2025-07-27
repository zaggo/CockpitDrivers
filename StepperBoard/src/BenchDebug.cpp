#include <BenchDebug.h>
const int kLedPin = 13;

BenchDebug::BenchDebug(BenchMode mode)
{
    Serial.begin(115200);
    Serial.println("Cockpit alive!");

    inputBuffer = "";

    if (mode & kGyroDrive)
    {
        gyroDrive = new GyroDrive();
        gyroDrive->homeAllAxis();
        Serial.println("GyroDrive initialized");
    }

    if (mode & kX25Motors)
    {
        x25Motors = new X25Motors();
        Serial.println("X25Motors initialized");
    }

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    delete gyroDrive;
    delete x25Motors;
}


const int kMaxCommandLength = 10;

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
                if (gyroDrive)
                {
                }
                if (x25Motors)
                {
                }
                if (commands[i] == "o") {
                    if (gyroDrive) {
                        gyroDrive->offAllAxes();
                    }
                    commandExecuted = true;
                    Serial.println("All motors off.");
                } else if (commands[i] == "h") {
                    if (gyroDrive) {
                        gyroDrive->homeAllAxis();
                    }
                    commandExecuted = true;
                    Serial.println("All motors homed.");
                }
            }

            if (!commandExecuted) {
                Serial.println("Unknown command. Type '?' for help.");
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

    if (gyroDrive)
    {
        gyroDrive->runAllAxes();
    }
    if (gyroDrive)
    {
        gyroDrive->runAllAxes();
    }
}