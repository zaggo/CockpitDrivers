#include <BenchDebug.h>
#if BENCHDEBUG
const int kLedPin = 13;

BenchDebug::BenchDebug(FuelGauge* fuelGauge): fuelGauge(fuelGauge)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("Fuelgauge BenchDebug"));

    inputBuffer = "";

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    fuelGauge->moveServo(leftTank, 0);    
    fuelGauge->moveServo(rightTank, 0);    

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleAltimeterInput(String command) {
    if (command.startsWith("lt")) {
        String rString = command.substring(2);
        rString.trim();
        float gallons = rString.toFloat();
        fuelGauge->moveServo(leftTank, gallons);
        return true;
    }  else if (command.startsWith("rt")) {
        String rString = command.substring(2);
        rString.trim();
        float gallons = rString.toFloat();
        fuelGauge->moveServo(rightTank, gallons);
        return true;
    }   else if (command.startsWith("cl")) {
        String rString = command.substring(2);
        rString.trim();
        float degree = rString.toFloat();
        fuelGauge->moveServo(leftTank, degree, true);
        return true;
    }   else if (command.startsWith("cr")) {
        String rString = command.substring(2);
        rString.trim();
        float degree = rString.toFloat();
        fuelGauge->moveServo(rightTank, degree, true);
        return true;
    }  else if (command.startsWith("br")) {
        String rString = command.substring(2);
        rString.trim();
        uint8_t brightness = rString.toInt();
        fuelGauge->setBrightness(brightness);
        return true;
    }  else if (command.startsWith("?")) {
        Serial.println(F("Fuel Gauge Commands:"));
        Serial.println(F("lt<gallons>: display fuel level left tank"));
        Serial.println(F("rt<gallons>: display fuel level right tank"));
        Serial.println(F("br<0..255>: set light brightness"));
        Serial.println(F("cl<degree>: calibrate left tank needle"));
        Serial.println(F("cr<degree>: calibrate right tank needle"));
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