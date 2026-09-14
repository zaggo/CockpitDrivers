#include <BenchDebug.h>
#if BENCHDEBUG

// No heartbeat LED here: the Nano's onboard LED is on D13, which is SPI SCK on
// this board. Driving it would fight the CAN link.

BenchDebug::BenchDebug(Altimeter* altimeter)
: altimeter(altimeter)
{
    Serial.begin(115200);
    Serial.println(F("Altimeter BenchDebug"));

    inputBuffer = "";

    altimeter->moveServo(flagServo, 0);

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
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
        printCalibration();
        return true;
    } else if (command.startsWith("zh")) {
        storeZero(hundred, F("100s"));
        return true;
    } else if (command.startsWith("zt")) {
        storeZero(thousand, F("1000s"));
        return true;
    } else if (command.startsWith("ze")) {
        storeZero(tenshousand, F("10ks"));
        return true;
    } else if (command.startsWith("cw")) {
        altimeter->wipeCalibration();
        Serial.println(F("Calibration wiped (needle zeros and baro)."));
        printCalibration();
        return true;
    } else if (command.startsWith("bn")) {
        String rString = command.substring(2);
        rString.trim();
        const uint16_t inHg100 = static_cast<uint16_t>(rString.toFloat() * 100. + 0.5);
        altimeter->setBaroCalibrationPoint(false, inHg100);
        Serial.print(F("Baro low point stored at raw "));
        Serial.println(altimeter->baroCalibration().low.raw);
        return true;
    } else if (command.startsWith("bx")) {
        String rString = command.substring(2);
        rString.trim();
        const uint16_t inHg100 = static_cast<uint16_t>(rString.toFloat() * 100. + 0.5);
        altimeter->setBaroCalibrationPoint(true, inHg100);
        Serial.print(F("Baro high point stored at raw "));
        Serial.println(altimeter->baroCalibration().high.raw);
        return true;
    } else if (command.startsWith("ba")) {
        Serial.print(F("Baro raw "));
        Serial.print(altimeter->baroRaw());
        Serial.print(F(" -> "));
        Serial.print(altimeter->baroInHg100Now() / 100.);
        Serial.println(F(" inHg"));
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
        Serial.println(F("zh / zt / ze: store current position as true zero (100s / 1000s / 10ks)"));
        Serial.println(F("cw: wipe calibration back to defaults"));
        Serial.println(F("bn<inHg>: store current pot position as the low baro point"));
        Serial.println(F("bx<inHg>: store current pot position as the high baro point"));
        Serial.println(F("ba: show current baro raw value and inHg"));
        printCalibration();
        return true;
    }
    return false;
}

void BenchDebug::storeZero(AltimeterAxis axis, const __FlashStringHelper* name)
{
    if (!altimeter->calibrateZero(axis)) {
        Serial.println(F("Not homed. Run 'ho' first."));
        return;
    }
    Serial.print(F("Stored zero for "));
    Serial.print(name);
    Serial.print(F(" axis, offset now "));
    Serial.print(altimeter->zeroAdjustDegree(axis));
    Serial.println(F(" degrees"));
}

void BenchDebug::printCalibration()
{
    Serial.println(F("Calibration:"));
    Serial.print(F("  zero 100s : ")); Serial.println(altimeter->zeroAdjustDegree(hundred));
    Serial.print(F("  zero 1000s: ")); Serial.println(altimeter->zeroAdjustDegree(thousand));
    Serial.print(F("  zero 10ks : ")); Serial.println(altimeter->zeroAdjustDegree(tenshousand));
    const BaroCalibration& baro = altimeter->baroCalibration();
    Serial.print(F("  baro low  : raw ")); Serial.print(baro.low.raw);
    Serial.print(F(" -> ")); Serial.print(baro.low.inHg100 / 100.);
    Serial.println(F(" inHg"));
    Serial.print(F("  baro high : raw ")); Serial.print(baro.high.raw);
    Serial.print(F(" -> ")); Serial.print(baro.high.inHg100 / 100.);
    Serial.println(F(" inHg"));
    Serial.print(F("  baro now  : raw ")); Serial.print(altimeter->baroRaw());
    Serial.print(F(" -> ")); Serial.print(altimeter->baroInHg100Now() / 100.);
    Serial.println(F(" inHg"));
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
    handleUserInput();
}
#endif