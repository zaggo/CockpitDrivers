#include <BenchDebug.h>
#if BENCHDEBUG
const int kLedPin = 13;

BenchDebug::BenchDebug(RPMGauge* rpmGauge, Odometer* odometer)
: rpmGauge(rpmGauge)
, odometer(odometer)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println(F("RPMGauge BenchDebug"));

    inputBuffer = "";

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    rpmGauge->moveNeedle(0);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleRPMGaugeInput(String command) {
    if (command.startsWith("rp")) {
        String rString = command.substring(2);
        rString.trim();
        float rpm = rString.toFloat();
        rpmGauge->moveNeedle(rpm);
        return true;
    } else if (command.startsWith("od")) {
        String rString = command.substring(2);
        rString.trim();
        float odometerValue = rString.toFloat();
        float digits[6];
        odometer->secondsToDigits(odometerValue, digits);
        odometer->displayNumber(digits);
        return true;
    } else if (command.startsWith("cl")) {
        String rString = command.substring(2);
        rString.trim();
        float degree = rString.toFloat();
        rpmGauge->moveNeedle(degree, true);
        return true;
    } else if (command.startsWith("br")) {
        String rString = command.substring(2);
        rString.trim();
        uint8_t brightness = rString.toInt();
        rpmGauge->setBrightness(brightness);
        return true;
    } else if (command.startsWith("?")) {
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
                    if (commandCount >= kMaxCommandLength - 1) {
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
                if (handleRPMGaugeInput(commands[i])) {
                    commandExecuted = true;
                }
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
