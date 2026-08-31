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

void BenchDebug::sendAirspeed() {
    byte data[4] = {0};
    // [0..1] IAS knots*10. [2..3] TAS stays 0 - the ASI's TAS ring is mechanical,
    // so AirspeedCAN ignores those bytes anyway.
    packBE16(data + 0, static_cast<uint16_t>(iasKnots * 10. + 0.5));

    Serial.println(String(F("Send Airspeed: ")) + iasKnots + F(" kts"));

    canBus->sendMessage(CanMessageId::airspeed, 4, data);
}

void BenchDebug::sendRpm() {
    byte data[2] = {0};
    packBE16(data + 0, rpmValue);

    Serial.println(String(F("Send RPM: ")) + rpmValue);

    canBus->sendMessage(CanMessageId::rpm, 2, data);
}

void BenchDebug::sendOdometer() {
    // Round to the nearest ten-thousandth-of-an-hour once (preserves up to 4
    // typed decimal digits, e.g. 123.4595), then extract every digit via
    // integer arithmetic - doing it digit-by-digit in floating point truncates
    // instead of rounds (823.3 -> tenths computed as 2.999... -> 2).
    //
    // The last two decimal digits (thousandths/ten-thousandths-of-an-hour)
    // don't map to a displayed digit - the device only shows whole
    // hundredths - but they set the fractional part of the CAN "hundredths"
    // field, which the device uses to roll the hundredths digit smoothly
    // between two values instead of snapping (see RPMGaugeCAN's
    // Odometer::displayNumber, which animates once that fraction is >= 0.9).
    uint32_t totalTenThousandths = static_cast<uint32_t>(odometerHours * 10000. + 0.5);

    uint32_t wholeHours = totalTenThousandths / 10000;
    uint16_t fracTenThousandths = static_cast<uint16_t>(totalTenThousandths % 10000);

    uint8_t d1000 = static_cast<uint8_t>((wholeHours / 1000) % 10);
    uint8_t d100 = static_cast<uint8_t>((wholeHours / 100) % 10);
    uint8_t d10 = static_cast<uint8_t>((wholeHours / 10) % 10);
    uint8_t d1 = static_cast<uint8_t>(wholeHours % 10);
    uint8_t dTenths = static_cast<uint8_t>(fracTenThousandths / 1000);
    uint16_t dHundredths100 = static_cast<uint16_t>(fracTenThousandths % 1000) * 10;

    byte data[7] = {0};
    data[0] = d1000;
    data[1] = d100;
    data[2] = d10;
    data[3] = d1;
    data[4] = dTenths;
    packBE16(data + 5, dHundredths100);

    Serial.println(String(F("Send Odometer hours: ")) + odometerHours);

    canBus->sendMessage(CanMessageId::odometer, 7, data);
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
    } else if (command.startsWith("rp")) {
        String rString = command.substring(2);
        rString.trim();
        rpmValue = static_cast<uint16_t>(rString.toInt());
        Serial.println(String(F("RPM set to "))+rpmValue);
        sendRpm();
        return true;
    } else if (command.startsWith("oh")) {
        String rString = command.substring(2);
        rString.trim();
        odometerHours = rString.toFloat();
        Serial.println(String(F("Odometer hours set to "))+odometerHours);
        sendOdometer();
        return true;
    } else if (command.startsWith("as")) {
        String rString = command.substring(2);
        rString.trim();
        iasKnots = rString.toFloat();
        Serial.println(String(F("Airspeed set to "))+iasKnots+F(" kts"));
        sendAirspeed();
        return true;
    } else if (command.startsWith("rw")) {
        startRudderWatch();
        return true;
    } else if (command.startsWith("?")) {
        Serial.println(F("DCU Commands:"));
        Serial.println(F("lt<kg>: display fuel level left tank"));
        Serial.println(F("rt<kg>: display fuel level right tank"));
        Serial.println(F("cl<0..255>: set light brightness"));
        Serial.println(F("rp<rpm>: set RPM gauge value"));
        Serial.println(F("oh<hours>: set odometer total hours"));
        Serial.println(F("as<knots>: set airspeed indicator (IAS)"));
        Serial.println(F("rw: watch rudder/toe brake input (any key stops)"));
        return true;
    }
    return false;
}

void BenchDebug::startRudderWatch()
{
    // Drop whatever the CAN layer holds from before the watch was armed, so only
    // frames that arrive while watching get printed.
    RudderToDcuMessage stale;
    canBus->takeRudderSample(stale);

    rudderWatchActive = true;
    rudderWatchPrinted = false;
    Serial.println(F("Rudder watch on - press any key to stop"));
}

void BenchDebug::stopRudderWatch()
{
    rudderWatchActive = false;
    Serial.println(F("Rudder watch off"));
}

void BenchDebug::handleRudderWatch()
{
    RudderToDcuMessage sample;
    if (!canBus->takeRudderSample(sample))
    {
        return;
    }

    // RudderCAN resends periodically even when the pedals are still - only the
    // changed triples are worth a line.
    if (rudderWatchPrinted &&
        sample.rudder == lastRudderPrinted.rudder &&
        sample.leftBrake == lastRudderPrinted.leftBrake &&
        sample.rightBrake == lastRudderPrinted.rightBrake)
    {
        return;
    }

    lastRudderPrinted = sample;
    rudderWatchPrinted = true;

    // No String here: heap churn in a frequently-hit print path is what froze the
    // CAN link on other AVR nodes.
    Serial.print(F("Rudder "));
    Serial.print(sample.rudder);
    Serial.print(F(" lBrk "));
    Serial.print(sample.leftBrake);
    Serial.print(F(" rBrk "));
    Serial.println(sample.rightBrake);
}

void BenchDebug::handleUserInput()
{
    static String inputBuffer = ""; // Zwischenspeicher für serielle Eingaben

    if (rudderWatchActive)
    {
        // Any key leaves the watch; the keystroke itself is not a command.
        if (Serial.available() > 0)
        {
            while (Serial.available() > 0)
            {
                Serial.read();
            }
            stopRudderWatch();
        }
        return;
    }

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

    if (rudderWatchActive)
    {
        handleRudderWatch();
    }
}
#endif