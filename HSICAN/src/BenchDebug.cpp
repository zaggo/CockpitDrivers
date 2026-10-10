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

// l = left needle = gs2Servo, r = right needle = gs1Servo; same order as the
// HsiConfig tables (index 0 = gs1, 1 = gs2).
const ServoId BenchDebug::kGsServoIds[2] = {gs1Servo, gs2Servo};

int BenchDebug::gsSideFromArg(const String &arg)
{
    if (arg == "r") return 0;
    if (arg == "l") return 1;
    return -1;
}

void BenchDebug::printGsTable(int side)
{
    const HSI::HsiConfig &cfg = hsi->calibration();
    Serial.print(side == 0 ? F("GS r (gs1): ") : F("GS l (gs2): "));
    for (uint8_t i = 0; i < cfg.gsPointCount[side]; i++) {
        Serial.print(cfg.gsPoints[side][i].dots1000 / 1000.0f, 2);
        Serial.print(F("->"));
        Serial.print(cfg.gsPoints[side][i].percent);
        Serial.print(F("% "));
    }
    Serial.println();
}

void BenchDebug::applyAll()
{
    hsi->applyFlags(flags);
    hsi->applyHeading(heading100, obs100, bug100);
    hsi->applyDeviation(cdiDots1000, gsDots1000);
}

void BenchDebug::resetHeadingMirror()
{
    // Homing puts the axes back at zero, so the mirror has to follow. Left
    // alone, the next hd/ob/bg reapplies the whole pre-homing picture and
    // drives the untouched axes back to where they stood before. Reset all
    // three even for a single-axis home: the obs and hdg axes are driven
    // relative to the heading, so a stale heading would shift them too.
    heading100 = 0;
    obs100 = 0;
    bug100 = 0;
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
    } else if (command == "hm" || command == "hm0") {
        resetHeadingMirror();
        hsi->beginHoming();
    } else if (command.length() == 3 && command.startsWith("hm")
               && command.charAt(2) >= '1' && command.charAt(2) < '1' + hsiAxisCount) {
        resetHeadingMirror();
        hsi->beginHoming(static_cast<HSIAxis>(command.charAt(2) - '1'));
    } else if (command == "hs") {
        hallMonitor = !hallMonitor;
        if (hallMonitor) {
            // Expect 0x70 in all three: GPB4..6 input, pulled up, inverted.
            Serial.print(F("Port B IODIR 0x"));
            Serial.print(hsi->readHallRegister(MCP23017Register::IODIR_B), HEX);
            Serial.print(F(" GPPU 0x"));
            Serial.print(hsi->readHallRegister(MCP23017Register::GPPU_B), HEX);
            Serial.print(F(" IPOL 0x"));
            Serial.println(hsi->readHallRegister(MCP23017Register::IPOL_B), HEX);
            lastHallBits = hsi->getHallBits();
            hallFlips = 0;
            lastHallPrintMs = 0; // print the current state right away
        }
        Serial.println(hallMonitor ? F("Hall monitor on - hs again to stop")
                                   : F("Hall monitor off"));
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
        // bl<axis> measures; bl<axis> <steps> sets directly. Compass and HDG
        // carry their hall mid-train, so the measurement misses the stages
        // after it and has to be topped up by hand.
        const int sp = command.indexOf(' ');
        if (sp < 0) {
            measureBacklash(axisFromArg(command.substring(2)));
        } else if (sp != 3 || !isDigit(command.charAt(sp + 1))) {
            Serial.println(F("usage: bl<o|c|h> [<steps>]"));
        } else {
            const HSIAxis axis = axisFromArg(command.substring(2));
            const uint16_t steps = static_cast<uint16_t>(command.substring(sp + 1).toInt());
            hsi->setStepsBacklash(axis, steps);
            Serial.print(F("Stored slack steps: "));
            Serial.println(steps);
        }
    } else if (command.startsWith("sa")) {
        // sa<servo> <deg100> <ratio1000>, e.g. "sa2 7100 2000" for gs1Servo.
        // Without this the EEPROM copies of the servo linkage could only ever
        // hold their compiled-in defaults, which is the whole reason they are
        // stored: a re-seated servo horn gets retuned here instead of reflashed.
        const int sp1 = command.indexOf(' ');
        const int sp2 = command.indexOf(' ', sp1 + 1);
        if (sp1 <= 2 || sp2 < 0) {
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
        // dpc <deg100PerDot>, e.g. "dpc 800" for 8.0 degrees per CDI dot.
        // The GS servos run from their taught tables (gp), not a factor.
        const int sp = command.indexOf(' ');
        if (sp < 0 || command.charAt(2) != 'c') {
            Serial.println(F("usage: dpc <deg100PerDot> (GS: use gm/ga/gp)"));
        } else {
            hsi->setDegPerDot(static_cast<int16_t>(command.substring(sp + 1).toInt()));
            Serial.println(F("deg per dot stored"));
        }
    } else if (command == "gm") {
        // Both GS servos to their absolute 50 % centre; ga moves from here.
        for (int i = 0; i < 2; i++) {
            gaPercent[i] = 0;
            hsi->setGsServoPercent(kGsServoIds[i], 0);
        }
        Serial.println(F("GS servos centred"));
    } else if (command.startsWith("gpc")) {
        const int i = gsSideFromArg(command.substring(3));
        if (i < 0) {
            Serial.println(F("usage: gpc<l|r>"));
        } else {
            hsi->clearGsPoints(kGsServoIds[i]);
            Serial.println(F("GS table cleared"));
        }
    } else if (command.startsWith("ga") || command.startsWith("gp")) {
        // ga<l|r> <-100..100>  absolute deflection from the 50 % centre
        // gp<l|r> <dots>       store (dots, current ga percent) for that servo
        const bool isPoint = command.charAt(1) == 'p';
        const int sp = command.indexOf(' ');
        const int i = sp < 0 ? -1 : gsSideFromArg(command.substring(2, sp));
        if (i < 0) {
            Serial.println(isPoint ? F("usage: gp<l|r> <dots>") : F("usage: ga<l|r> <-100..100>"));
        } else if (isPoint) {
            const int16_t dots1000 = static_cast<int16_t>(command.substring(sp + 1).toFloat() * 1000.0f);
            if (!hsi->addGsPoint(kGsServoIds[i], dots1000, gaPercent[i])) {
                Serial.println(F("GS table full - gpc<l|r> clears it"));
            } else {
                printGsTable(i);
            }
        } else {
            const long pct = command.substring(sp + 1).toInt();
            if (pct < -100 || pct > 100) {
                Serial.println(F("percent out of range -100..100"));
            } else {
                gaPercent[i] = static_cast<int8_t>(pct);
                hsi->setGsServoPercent(kGsServoIds[i], gaPercent[i]);
            }
        }
    } else if (command.startsWith("?")) {
        Serial.println(F("HSI Commands (send hm first - hd/ob/bg do nothing until homed):"));
        Serial.println(F("hd<deg> heading   ob<deg> OBS      bg<deg> HDG bug"));
        Serial.println(F("cd<dots> CDI      gs<dots> GS      fl<bits> flags"));
        Serial.println(F("hm[0] home all    hm1 OBS  hm2 compass  hm3 HDG"));
        Serial.println(F("li<0-255> light   st stop axes     hs hall monitor"));
        Serial.println(F("axis letters: o=OBS pointer  c=compass card  h=HDG bug"));
        Serial.println(F("cz<o|c|h> calibrate zero           bl<o|c|h> measure backlash"));
        Serial.println(F("blsave store measured backlash     bl<o|c|h> <steps> set backlash"));
        Serial.println(F("cw wipe calibration"));
        Serial.println(F("sa<servo> <deg100> <ratio1000> servo adjust (CDI, TO/FROM)"));
        Serial.println(F("dpc <deg100PerDot> degrees per CDI dot"));
        Serial.println(F("GS servos (l=left=gs2, r=right=gs1):"));
        Serial.println(F("gm centre both   ga<l|r> <-100..100> deflect   gp<l|r> <dots> store point"));
        Serial.println(F("gpc<l|r> clear table"));
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

void BenchDebug::monitorHalls()
{
    // Count every change loop() sees, but print at most every 250 ms: a
    // floating input can flip thousands of times a second, and printing each
    // flip would stall the steppers.
    const uint8_t bits = hsi->getHallBits();
    if (bits != lastHallBits) {
        lastHallBits = bits;
        hallFlips++;
    }
    const uint32_t now = millis();
    if (lastHallPrintMs != 0 && (hallFlips == 0 || now - lastHallPrintMs < 250)) {
        return;
    }
    lastHallPrintMs = now == 0 ? 1 : now;
    Serial.print(F("Hall obs:"));
    Serial.print((bits & kHallBits[obs]) ? 1 : 0);
    Serial.print(F(" compass:"));
    Serial.print((bits & kHallBits[compass]) ? 1 : 0);
    Serial.print(F(" hdg:"));
    Serial.print((bits & kHallBits[hdg]) ? 1 : 0);
    Serial.print(F("  flips:"));
    Serial.println(hallFlips);
    hallFlips = 0;
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
    if (hallMonitor) {
        monitorHalls();
    }

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
