#include <BenchDebug.h>
#if BENCHDEBUG
const int kLedPin = 13;

BenchDebug::BenchDebug()
{
    Serial.begin(115200);
    Serial.println(F("Altimeter startup…"));

    inputBuffer = "";

    altimeter = new Altimeter();
    //hsi->homeAllAxis();
    Serial.println(F("Altimeter initialized"));

    // hsi->moveToDegree(0,0,0,0,HSI::FromTo::from,0);
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    altimeter->moveServo(flagServo, 0);    

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
    delete altimeter;
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleAltimeterInput(String command) {
    if (command.startsWith("hu")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        altimeter->moveToDegree(hundred, degrees);
        return true;
    }  else if (command.startsWith("ho")) {
        String rString = command.substring(2);
        rString.trim();
        if (rString.length() == 0) {
            altimeter->homeAllAxis();
            return true;
        }
        AltimeterAxis axis = static_cast<AltimeterAxis>(rString.toInt());
        altimeter->homeAxis(axis);
        return true;
    } else if (command.startsWith("th")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        altimeter->moveToDegree(thousand, degrees);
        return true;
    } else if (command.startsWith("te")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        altimeter->moveToDegree(tenshousand, degrees);
        return true;
    } else if (command.startsWith("fl")) {
        String rString = command.substring(2);
        rString.trim();
        float value = rString.toFloat();
        double flagDegree = kServoMinimumDegree[flagServo] + (kServoMaximumDegree[flagServo] - kServoMinimumDegree[flagServo]) * constrain(value, 0.0f, 1.0f);
        altimeter->moveServo(flagServo, flagDegree);
        return true;
    } else if (command.startsWith("cf")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        altimeter->moveServo(flagServo, degrees, true);
        return true;
    } else if (command.startsWith("he")) {
        String rString = command.substring(2);
        rString.trim();
        float feet = rString.toFloat();
        Serial.println("Moving to height: " + String(feet) + " feet.");
        altimeter->moveToHeight(feet);
        currentHeightInFeet = feet;
        return true;
    } else if (command.startsWith("st")) {
        Serial.println(F("Altimeter Status:"));
        Serial.println(String(F("  Homed: ")) + String(altimeter->isHomed ? F("true") : F("false")));
        Serial.println(String(F("  Current Height: ")) + String(altimeter->currentHeightInFeet(), 2) + String(F(" feet")));
        return true;
    }  else if (command.startsWith("?")) {
        Serial.println(F("Altimeter Commands:"));
        Serial.println(F("ho<axis>: Home axis, no axis = all, 0 = hundreds, 1 = 1k, 2 = 10k."));
        Serial.println(F("hu<degrees>: Move the hundreds axis to the given degrees."));
        Serial.println(F("th<degrees>: Move the thousands axis to the given degrees."));
        Serial.println(F("te<degrees>: Move the 10k axis to the given degrees."));
        Serial.println(F("fl<value>: 0 means fully 'off', 1 means fully 'on'."));
        Serial.println(F("cf<value>: calibrate flag servo to given degrees."));
        Serial.println(F("he<feet>: move altimeter to given height in feet.")); 
        Serial.println(F("st: shows current altimeter status."));
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
    altimeter->loop();

    // Fetch and display pressure ratio if changed
    if(millis() - fetchPressureRatio > 500L) {
        fetchPressureRatio = millis();
        float ratio = altimeter->fetchPressureRatio();
        if(round(ratio*100.) != round(lastPressureRatio*100.)) {
            lastPressureRatio = ratio;
            Serial.print(F("Pressure Ratio: "));
            Serial.println(lastPressureRatio, 3);
        }
    }
}
#endif