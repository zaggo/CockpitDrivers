#include <BenchDebug.h>
#if BENCHDEBUG

// No heartbeat LED here: the Nano's onboard LED is on D13, which is SPI SCK on
// this board. Driving it would fight the CAN link.

BenchDebug::BenchDebug(WhiskeyCompass* compass)
: compass(compass)
{
    Serial.begin(115200);
    Serial.println(F("WhiskeyCompass BenchDebug"));

    inputBuffer = "";

    Serial.println(F("System running! '?' for commands"));
}

void BenchDebug::printStatus()
{
    Serial.println(String(F("homed: ")) + (compass->isHomed ? F("yes") : F("no"))
                 + F(" homing: ") + (compass->isHoming() ? F("yes") : F("no"))
                 + F(" position: ") + String(compass->position())
                 + F(" zeroAdjust: ") + String(compass->zeroAdjustDegree()));
}

// One command per line, unlike the DCU console - this board has few enough of
// them that a splitter would be more code than it saves.
bool BenchDebug::handleCommand(String command)
{
    if (command.startsWith("hd")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        if (compass->moveToHeading(degrees) == WhiskeyCompass::notHomed) {
            Serial.println(F("Not homed - run 'ho' first"));
            return true;
        }
        Serial.println(String(F("Heading set to ")) + degrees + F(" deg"));
        return true;
    } else if (command.startsWith("ho")) {
        compass->beginHoming();
        Serial.println(F("Homing started"));
        return true;
    } else if (command.startsWith("li")) {
        String rString = command.substring(2);
        rString.trim();
        long value = rString.toInt();
        uint8_t pwm = (uint8_t)constrain(value, 0L, 255L);
        compass->setBrightness(pwm);
        Serial.println(String(F("Brightness set to ")) + pwm);
        return true;
    } else if (command.startsWith("cz")) {
        if (compass->calibrateZero()) {
            Serial.println(String(F("Zero stored, offset now "))
                         + compass->zeroAdjustDegree() + F(" deg"));
        } else {
            Serial.println(F("Not homed - cannot store zero"));
        }
        return true;
    } else if (command.startsWith("cw")) {
        compass->wipeCalibration();
        Serial.println(F("Calibration wiped to defaults"));
        return true;
    } else if (command.startsWith("st")) {
        compass->stop();
        compass->off();
        Serial.println(F("Card stopped and de-energised"));
        return true;
    } else if (command.startsWith("sd")) {
        printStatus();
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("WhiskeyCompass Commands:"));
        Serial.println(F("ho: start homing"));
        Serial.println(F("hd<deg>: move card to magnetic heading"));
        Serial.println(F("li<0..255>: set LED brightness"));
        Serial.println(F("cz: store current position as north"));
        Serial.println(F("cw: wipe calibration to defaults"));
        Serial.println(F("st: stop and de-energise the card"));
        Serial.println(F("sd: show status"));
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
            if (inputBuffer.length() > 0 && !handleCommand(inputBuffer))
            {
                Serial.println(String(F("Unknown command: ")) + inputBuffer);
            }
            inputBuffer = "";
        }
        else if (receivedChar != '\r')
        {
            inputBuffer += receivedChar;
            Serial.print(receivedChar);
        }
    }
}

void BenchDebug::loop()
{
    handleUserInput();
}
#endif
