#include <BenchDebug.h>
#if BENCHDEBUG
const int kLedPin = 13;

BenchDebug::BenchDebug()
{
    Serial.begin(115200);
    Serial.println(F("HSI startup…"));

    inputBuffer = "";

    hsi = new HSI();
    //hsi->homeAllAxis();
    Serial.println(F("HSIDrive initialized"));

    // hsi->moveToDegree(0,0,0,0,HSI::FromTo::from,0);
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    hsi->moveServo(fromToServo, 0);
    hsi->moveServo(vsi1Servo, 0);
    hsi->moveServo(vsi2Servo, 0);
    hsi->moveServo(vorServo, 0);
    

    Serial.println(F("System running!"));
}

BenchDebug::~BenchDebug()
{
    delete hsi;
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleHSIInput(String command) {
    if (command.startsWith("cd")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveToDegree(cdi, degrees);
        return true;
    } else if (command.startsWith("sd")) {
        String rString = command.substring(2);
        rString.trim();
        int32_t steps = rString.toInt();
        hsi->moveSteps(cdi, steps, true);
        return true;
    } else if (command.startsWith("ho")) {
        String rString = command.substring(2);
        rString.trim();
        if (rString.length() == 0) {
            hsi->homeAllAxis();
            return true;
        }
        HSIAxis axis = static_cast<HSIAxis>(rString.toInt());
        hsi->homeAxis(axis);
        return true;
    } else if (command.startsWith("co")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveToDegree(compass, degrees);
        return true;
    } else if (command.startsWith("so")) {
        String rString = command.substring(2);
        rString.trim();
        int32_t steps = rString.toInt();
        hsi->moveSteps(compass, steps, true);
        return true;
    } else if (command.startsWith("hd")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveToDegree(hdg,-degrees);
        return true;
    } else if (command.startsWith("sh")) {
        String rString = command.substring(2);
        rString.trim();
        int32_t steps = rString.toInt();
        hsi->moveSteps(hdg, steps, true);
        return true;
    } else if (command.startsWith("vo")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveServo(vorServo, degrees);
        return true;
    } else if (command.startsWith("gs")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveServo(vsi1Servo, degrees);
        hsi->moveServo(vsi2Servo, degrees);
        return true;
    } else if (command.startsWith("ga")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveServo(vsi1Servo, degrees);
        return true;
    } else if (command.startsWith("gb")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveServo(vsi2Servo, degrees);
        return true;
    } else if (command.startsWith("ft")) {
        String rString = command.substring(2);
        rString.trim();
        int fromTo = rString.toInt();

        HSI::FromTo fromToValue;
        if (fromTo == 0) {
            fromToValue = HSI::FromTo::noNav;
        } else if (fromTo == 1) {
            fromToValue = HSI::FromTo::from;
        } else if (fromTo == 2) {
            fromToValue = HSI::FromTo::to;
        } else {
            Serial.println("Invalid value for 'ft' command. Use 0, 1 or 2.");
            return false;
        }
        hsi->moveServo(fromToServo, fromToValue);
        return true;
    } else if (command.startsWith("xd")) {
        String rString = command.substring(2);
        rString.trim();
        double degrees = rString.toDouble();
        combiCdiDegree = degrees;
        hsi->moveToDegree(combiCdiDegree, combiCompassDegree, combiHdgDegree, 0, HSI::FromTo::noNav, 0);
        return true;
    } else if (command.startsWith("xo")) {
        String rString = command.substring(2);
        rString.trim();
        double degrees = rString.toDouble();
        combiCompassDegree = degrees;
        hsi->moveToDegree(combiCdiDegree, combiCompassDegree, combiHdgDegree, 0, HSI::FromTo::noNav, 0);
        return true;
    } else if (command.startsWith("xh")) {
        String rString = command.substring(2);
        rString.trim();
        double degrees = rString.toDouble();
        combiHdgDegree = degrees;
        hsi->moveToDegree(combiCdiDegree, combiCompassDegree, combiHdgDegree, 0, HSI::FromTo::noNav, 0);
        return true;  
    } else if (command.startsWith("?")) {
        Serial.println(F("HSI Commands:"));
        Serial.println(F("ho <axis>: Home axis, no axis = all, 0 = CDI, 1 = Comp, 2 = HDG."));
        Serial.println(F("cd <degrees>: Move the CDI axis to the given degrees."));
        Serial.println(F("co <degrees>: Move the Compass axis to the given degrees."));
        Serial.println(F("hd <degrees>: Move the Heading axis to the given degrees."));
        Serial.println(F("vo <degrees>: Move the VOR needle servo by the given degrees."));
        Serial.println(F("gs <degrees>: Move the Glidescope needle servo by the given degrees."));
        Serial.println(F("ga <degrees>: Move ONLY the Glidescope needle servo 1 by the given degrees."));
        Serial.println(F("gb <degrees>: Move OMLY the Glidescope needle servo 2 by the given degrees."));
        Serial.println(F("ft <value>: 0 means 'no nav', 1 means 'from', 2 means 'to'."));
        Serial.println(F("sd <steps>: Move the CDI axis by the given steps (positive or negative)."));
        Serial.println(F("so <steps>: Move the Compass axis by the given steps (positive or negative)."));
        Serial.println(F("sh <steps>: Move the HDG axis by the given steps (positive or negative)."));
        Serial.println(F("xd <degrees>: Move CDI to degrees in combined mode."));
        Serial.println(F("xo <degrees>: Move Compass to degrees in combined mode."));
        Serial.println(F("xh <degrees>: Move HDG to degrees in combined mode."));
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
                commandExecuted = handleHSIInput(commands[i]);
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

    int16_t cdiEncoder = hsi->getCdiEncoder();
    if (degCDI != (double)cdiEncoder) {
        degCDI = (double)cdiEncoder;
        hsi->moveToDegree(cdi, -degCDI);
        Serial.print(F("Bench Debug CDIEncoder: "));
        Serial.println(degCDI);
    }
    int16_t compEncoder = hsi->getCompEncoder();
    if (degCompass != (double)compEncoder) {
        degCompass = (double)compEncoder;
        hsi->moveToDegree(hdg, -degCompass);
        Serial.print(F("Bench Debug CompassEncoder: "));
        Serial.println(degCompass);
    }
}
#endif