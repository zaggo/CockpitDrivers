#include <BenchDebug.h>
#if BENCHDEBUG
#include <math.h>
const int kLedPin = 13;

BenchDebug::BenchDebug(HSI* hsi) : hsi(hsi)
{
    pinMode(kLedPin, OUTPUT);
    Serial.begin(115200);
    Serial.println(F("BenchDebug ready - send ? for commands"));
}

BenchDebug::~BenchDebug()
{
    // main.cpp owns the HSI instance; do not delete it here.
}

uint16_t BenchDebug::wrapDeg100(float degrees)
{
    float wrapped = fmodf(degrees, 360.0f);
    if (wrapped < 0.0f)
    {
        wrapped += 360.0f;
    }
    uint32_t deg100 = static_cast<uint32_t>(wrapped * 100.0f + 0.5f);
    return deg100 > 35999 ? 0 : static_cast<uint16_t>(deg100);
}

HSIAxis BenchDebug::axisFromArg(const String &arg)
{
    if (arg.startsWith("c"))
    {
        return compass;
    }
    if (arg.startsWith("h"))
    {
        return hdg;
    }
    return obs;
}

void BenchDebug::applyAll()
{
    hsi->applyFlags(flags);
    hsi->applyHeading(heading100, obs100, bug100);
    hsi->applyDeviation(cdiDots1000, gsDots1000);
}

void BenchDebug::measureBacklash(HSIAxis axis)
{
    Serial.println(F("Measuring backlash - do not touch the panel"));

    // Measure the raw mechanism, so the compensation must be off first.
    hsi->setStepsBacklash(axis, 0);
    pendingBacklash = hsi->measureBacklashSteps(axis);
    pendingBacklashAxis = axis;

    Serial.print(F("Measured slack steps: "));
    Serial.println(pendingBacklash);
    Serial.println(F("Send blsave to store it"));
}

bool BenchDebug::handleHSIInput(String command) {
    if (command.startsWith("hd")) {
        heading100 = wrapDeg100(command.substring(2).toFloat());
        applyAll();
    } else if (command.startsWith("ob")) {
        obs100 = wrapDeg100(command.substring(2).toFloat());
        applyAll();
    } else if (command.startsWith("bg")) {
        bug100 = wrapDeg100(command.substring(2).toFloat());
        applyAll();
    } else if (command.startsWith("fl")) {
        flags = static_cast<uint8_t>(command.substring(2).toInt()) & 0x07;
        applyAll();
    } else if (command.startsWith("cd")) {
        cdiDots1000 = static_cast<int16_t>(command.substring(2).toFloat() * 1000.0f);
        applyAll();
    } else if (command.startsWith("gs")) {
        gsDots1000 = static_cast<int16_t>(command.substring(2).toFloat() * 1000.0f);
        applyAll();
    } else if (command.startsWith("li")) {
        hsi->setBrightness(static_cast<uint8_t>(command.substring(2).toInt()));
    } else if (command == "hm") {
        hsi->beginHoming();
    } else if (command == "st") {
        hsi->stopAllAxes();
        hsi->offAllAxes();
    } else if (command == "cw") {
        hsi->wipeCalibration();
        Serial.println(F("Calibration wiped"));
    } else if (command.startsWith("cz")) {
        HSIAxis axis = axisFromArg(command.substring(2));
        hsi->calibrateZero(axis);
    } else if (command == "blsave") {
        hsi->setStepsBacklash(pendingBacklashAxis, static_cast<uint16_t>(pendingBacklash));
        Serial.print(F("Stored slack steps: "));
        Serial.println(pendingBacklash);
    } else if (command.startsWith("bl")) {
        measureBacklash(axisFromArg(command.substring(2)));
    } else if (command.startsWith("sa")) {
        // sa<servo> <deg100> <ratio1000>, e.g. "sa2 7100 2000" for gs1Servo.
        // Without this the EEPROM copies of the servo linkage could only ever
        // hold their compiled-in defaults, which is the whole reason they are
        // stored: a re-seated servo horn gets retuned here instead of reflashed.
        const int sp1 = command.indexOf(' ');
        const int sp2 = command.indexOf(' ', sp1 + 1);
        if (sp1 < 0 || sp2 < 0) {
            Serial.println(F("usage: sa<servo> <deg100> <ratio1000>"));
        } else {
            const int id = command.substring(2, sp1).toInt();
            if (id < 0 || id >= servoCount) {
                Serial.println(F("bad servo index"));
            } else {
                hsi->setServoAdjust(static_cast<ServoId>(id),
                                    static_cast<int16_t>(command.substring(sp1 + 1, sp2).toInt()),
                                    static_cast<int16_t>(command.substring(sp2 + 1).toInt()));
                Serial.println(F("servo adjust stored"));
            }
        }
    } else if (command.startsWith("dp")) {
        // dp<c|g> <deg100PerDot>, e.g. "dpg 800" for 8.0 degrees per GS dot.
        const int sp = command.indexOf(' ');
        if (sp < 0) {
            Serial.println(F("usage: dp<c|g> <deg100PerDot>"));
        } else {
            const bool isGs = command.charAt(2) == 'g';
            hsi->setDegPerDot(isGs, static_cast<int16_t>(command.substring(sp + 1).toInt()));
            Serial.println(F("deg per dot stored"));
        }
    } else if (command.startsWith("?")) {
        Serial.println(F("HSI Commands (send hm first - hd/ob/bg do nothing until homed):"));
        Serial.println(F("hd<deg> heading   ob<deg> OBS      bg<deg> HDG bug"));
        Serial.println(F("cd<dots> CDI      gs<dots> GS      fl<bits> flags"));
        Serial.println(F("hm home           li<0-255> light  st stop axes"));
        Serial.println(F("cz<o|c|h> calibrate zero           bl<o|c|h> measure backlash"));
        Serial.println(F("blsave store measured backlash     cw wipe calibration"));
        Serial.println(F("sa<servo> <deg100> <ratio1000> servo adjust"));
        Serial.println(F("dp<c|g> <deg100PerDot> degrees per deviation dot"));
    } else {
        return false;
    }
    return true;
}

void BenchDebug::handleUserInput()
{
    // One command per line, spaces included: sa/dp take space-separated
    // arguments of their own, so a line can no longer be split on spaces into
    // several chained commands the way the old CDI/Compass/HDG jog commands
    // allowed.
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read(); // Einzelnes Zeichen lesen
        if (receivedChar == '\n')
        {                       // Enter erkannt
            Serial.println();   // Neue Zeile
            inputBuffer.trim(); // Eingabe bereinigen (Leerzeichen etc.)

            if (inputBuffer.length() > 0 && !handleHSIInput(inputBuffer)) {
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

double degCDI= 0.;
double degCompass= 0.;

void BenchDebug::loop()
{
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    handleUserInput();
    hsi->loop();

    int16_t obsEncoder = hsi->getObsEncoder();
    if (degCDI != (double)obsEncoder) {
        degCDI = (double)obsEncoder;
        hsi->moveToDegree(obs, -degCDI);
        Serial.print(F("OBS encoder: "));
        Serial.println(degCDI);
    }
    int16_t hdgEncoder = hsi->getHdgEncoder();
    if (degCompass != (double)hdgEncoder) {
        degCompass = (double)hdgEncoder;
        hsi->moveToDegree(hdg, -degCompass);
        Serial.print(F("HDG encoder: "));
        Serial.println(degCompass);
    }
}
#endif
