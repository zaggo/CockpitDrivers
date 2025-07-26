#include <BenchDebug.h>
const int kLedPin = 13;

BenchDebug::BenchDebug(BenchMode mode)
{
    Serial.begin(115200);
    Serial.println("Cockpit alive!");

    inputBuffer = "";

    if (mode & kGyroDrive)
    {
        gyroDrive = new GyroDrive();
        gyroDrive->homeAllAxis();
        Serial.println("GyroDrive initialized");
    }

    if (mode & kX25Motors)
    {
        x25Motors = new X25Motors();
        Serial.println("X25Motors initialized");
    }

    if (mode & kHSI)
    {
        hsi = new HSI();
        hsi->homeAllAxis();
        Serial.println("HSIDrive initialized");
    }

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    delete gyroDrive;
    delete x25Motors;
    delete hsi;
}


const int kMaxCommandLength = 10;


bool BenchDebug::handleHSIInput(String command) {
    // There are multiple known commands, starting with a single character or two character combination.
    // Some of the commands have an additional parameter, which is a integer or float value.
    // The commands are:
    // - 'vr' followed by a float value: Move the VOR axis to the given degrees.
    // - 'cr' followed by a float value: Move the Compass axis to the given degrees.
    // - 'br' followed by a float value: Move the Bug axis to the given degrees.
    // - 'vo' followed by a float value: Move the VOR needle servo by the given degrees.
    // - 'gs' followed by a float value: Move the Glidescope needle servo by the given degrees.
    // - 'ft' followed by a int value: 0 means "no nav", 1 means "from", 2 means "to".
    // - '?': Print this help message.
    // If the command is not recognized, false is returned. Otherwise, true is returned.

    if (command.startsWith("vr")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveToDegree(HSIAxis::vor, degrees);
        return true;
    } else if (command.startsWith("cr")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveToDegree(HSIAxis::compass, degrees);
        return true;
    } else if (command.startsWith("br")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveToDegree(HSIAxis::bug, degrees);
        return true;
    } else if (command.startsWith("vo")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveServo(HSI::vorServo, degrees);
        return true;
    } else if (command.startsWith("gs")) {
        String rString = command.substring(2);
        rString.trim();
        float degrees = rString.toFloat();
        hsi->moveServo(HSI::gsServo, degrees);
        return true;
    } else if (command.startsWith("ft")) {
        String rString = command.substring(2);
        rString.trim();
        int fromTo = rString.toInt();
        HSI::FromTo fromToValue;
        if (fromTo == 0) {
            fromToValue = HSI::noNav;
        } else if (fromTo == 1) {
            fromToValue = HSI::from;
        } else if (fromTo == 2) {
            fromToValue = HSI::to;
        } else {
            Serial.println("Invalid value for 'ft' command. Use 0, 1 or 2.");
            return false;
        }
        hsi->moveServo(HSI::fromToServo, fromToValue);
        return true;
    } else if (command.startsWith("?")) {
        Serial.println("HSI Commands:");
        Serial.println("vr <degrees>: Move the VOR axis to the given degrees.");
        Serial.println("cr <degrees>: Move the Compass axis to the given degrees.");
        Serial.println("br <degrees>: Move the Bug axis to the given degrees.");
        Serial.println("vo <degrees>: Move the VOR needle servo by the given degrees.");
        Serial.println("gs <degrees>: Move the Glidescope needle servo by the given degrees.");
        Serial.println("ft <value>: 0 means 'no nav', 1 means 'from', 2 means 'to'.");
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

                if (hsi) {
                    commandExecuted = handleHSIInput(commands[i]);
                }
                if (gyroDrive)
                {
                }
                if (x25Motors)
                {
                }
                if (commands[i] == "o") {
                    if (gyroDrive) {
                        gyroDrive->offAllAxes();
                    }
                    if (hsi) {
                        hsi->offAllAxes();
                    }
                    commandExecuted = true;
                    Serial.println("All motors off.");
                } else if (commands[i] == "h") {
                    if (gyroDrive) {
                        gyroDrive->homeAllAxis();
                    }
                    if (hsi) {
                        hsi->homeAllAxis();
                    }
                    commandExecuted = true;
                    Serial.println("All motors homed.");
                }
            }

            if (!commandExecuted) {
                Serial.println("Unknown command. Type '?' for help.");
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

    if (gyroDrive)
    {
        gyroDrive->runAllAxes();
    }
    if (gyroDrive)
    {
        gyroDrive->runAllAxes();
    }
    if (hsi)
    {
        hsi->runAllAxes();
    }
}