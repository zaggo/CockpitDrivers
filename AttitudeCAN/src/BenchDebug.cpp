#include <BenchDebug.h>
#if BENCHDEBUG

// No heartbeat LED here: the Nano's onboard LED is on D13, which is SPI SCK on
// this board. Driving it would fight the CAN link.

BenchDebug::BenchDebug(AttitudeIndicator* attitude)
: attitude(attitude)
{
    Serial.begin(115200);
    Serial.println(F("AttitudeIndicator BenchDebug"));

    inputBuffer = "";

    Serial.println(F("System running! '?' for commands"));
}

void BenchDebug::printStatus()
{
    Serial.println(String(F("homed: ")) + (attitude->isHomed ? F("yes") : F("no"))
                 + F(" homing: ") + (attitude->isHoming() ? F("yes") : F("no"))
                 + F(" homingFailed: ") + (attitude->hasHomingFailed() ? F("yes") : F("no")));
    Serial.println(String(F("motor pos roll/pitch: ")) + String(attitude->position(roll))
                 + F("/") + String(attitude->position(pitch)));
    Serial.println(String(F("display roll/pitch deg: ")) + String(attitude->displayRollDegree(), 1)
                 + F("/") + String(attitude->displayPitchDegree(), 1));
    Serial.println(String(F("hall roll/pitch: ")) + (attitude->hallActive(roll) ? F("ON") : F("OFF"))
                 + F("/") + (attitude->hallActive(pitch) ? F("ON") : F("OFF")));
    Serial.println(String(F("zeroAdjust roll/pitch: ")) + String(attitude->zeroAdjustDegree(roll))
                 + F("/") + String(attitude->zeroAdjustDegree(pitch)));
}

// Prints both Hall states on every change until a line arrives on Serial.
// Turn the axes by hand to find the sensor windows.
void BenchDebug::hallMonitor()
{
    Serial.println(F("Hall monitor - press Enter to stop"));
    bool lastRoll = attitude->hallActive(roll);
    bool lastPitch = attitude->hallActive(pitch);
    Serial.println(String(F("roll ")) + (lastRoll ? F("ON") : F("OFF"))
                 + F(" pitch ") + (lastPitch ? F("ON") : F("OFF")));
    while (true)
    {
        while (Serial.available() > 0)
        {
            const char c = Serial.read();
            if (c == '\n')
            {
                Serial.println(F("Hall monitor stopped"));
                return;
            }
        }
        const bool r = attitude->hallActive(roll);
        const bool p = attitude->hallActive(pitch);
        if (r != lastRoll || p != lastPitch)
        {
            lastRoll = r;
            lastPitch = p;
            Serial.println(String(F("roll ")) + (r ? F("ON") : F("OFF"))
                         + F(" pitch ") + (p ? F("ON") : F("OFF")));
        }
        delay(5);
    }
}

static bool reportMove(AttitudeIndicator::AttitudeResult result)
{
    if (result == AttitudeIndicator::notHomed)
    {
        Serial.println(F("Not homed - run 'ho' first"));
        return false;
    }
    return true;
}

// One command per line, unlike the DCU console - this board has few enough of
// them that a splitter would be more code than it saves.
bool BenchDebug::handleCommand(String command)
{
    if (command.startsWith("ho")) {
        attitude->beginHoming();
        Serial.println(F("Homing started"));
        return true;
    } else if (command.startsWith("ro")) {
        String rString = command.substring(2);
        rString.trim();
        if (rString.length() == 0) {
            Serial.println(F("Usage: ro<deg>, e.g. ro-15"));
            return true;
        }
        const float degrees = rString.toFloat();
        if (reportMove(attitude->moveAxisToDegree(roll, degrees))) {
            Serial.println(String(F("Roll set to ")) + degrees + F(" deg"));
        }
        return true;
    } else if (command.startsWith("pi")) {
        String rString = command.substring(2);
        rString.trim();
        if (rString.length() == 0) {
            Serial.println(F("Usage: pi<deg>, e.g. pi10 (clamped to +-30)"));
            return true;
        }
        const float degrees = rString.toFloat();
        if (reportMove(attitude->moveAxisToDegree(pitch, degrees))) {
            Serial.println(String(F("Pitch set to ")) + degrees + F(" deg"));
        }
        return true;
    } else if (command.startsWith("at")) {
        String rString = command.substring(2);
        rString.trim();
        const int comma = rString.indexOf(',');
        if (comma < 0) {
            Serial.println(F("Usage: at<pitch>,<roll>, e.g. at10,-20"));
            return true;
        }
        const float pitchDeg = rString.substring(0, comma).toFloat();
        const float rollDeg = rString.substring(comma + 1).toFloat();
        if (reportMove(attitude->moveToAttitude(pitchDeg, rollDeg))) {
            Serial.println(String(F("Attitude set to pitch ")) + pitchDeg + F(" roll ") + rollDeg);
        }
        return true;
    } else if (command.startsWith("zr")) {
        if (attitude->calibrateZero(roll)) {
            Serial.println(String(F("Roll zero stored, offset now ")) + attitude->zeroAdjustDegree(roll) + F(" deg"));
        } else {
            Serial.println(F("Not homed - cannot store zero"));
        }
        return true;
    } else if (command.startsWith("zp")) {
        if (attitude->calibrateZero(pitch)) {
            Serial.println(String(F("Pitch zero stored, offset now ")) + attitude->zeroAdjustDegree(pitch) + F(" deg"));
        } else {
            Serial.println(F("Not homed - cannot store zero"));
        }
        return true;
    } else if (command.startsWith("cw")) {
        attitude->wipeCalibration();
        Serial.println(F("Calibration wiped to defaults - re-home with 'ho'"));
        return true;
    } else if (command.startsWith("li")) {
        String rString = command.substring(2);
        rString.trim();
        long value = rString.toInt();
        uint8_t pwm = (uint8_t)constrain(value, 0L, 255L);
        attitude->setBrightness(pwm);
        Serial.println(String(F("Brightness set to ")) + pwm);
        return true;
    } else if (command.startsWith("mo")) {
        attitude->stop();
        attitude->off();
        Serial.println(F("Motors stopped and de-energised"));
        return true;
    } else if (command.startsWith("st")) {
        printStatus();
        return true;
    } else if (command.startsWith("hm")) {
        hallMonitor();
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("AttitudeIndicator Commands:"));
        Serial.println(F("ho: start homing (roll first, then pitch)"));
        Serial.println(F("ro<deg>: roll to degrees (display frame)"));
        Serial.println(F("pi<deg>: pitch to degrees (display frame, clamped +-30)"));
        Serial.println(F("at<pitch>,<roll>: setpoint as sent by the bus"));
        Serial.println(F("zr / zp: store current roll / pitch position as zero"));
        Serial.println(F("cw: wipe calibration to defaults"));
        Serial.println(F("li<0..255>: set LED brightness"));
        Serial.println(F("mo: stop and de-energise both motors"));
        Serial.println(F("st: show status"));
        Serial.println(F("hm: hall monitor (Enter stops)"));
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
