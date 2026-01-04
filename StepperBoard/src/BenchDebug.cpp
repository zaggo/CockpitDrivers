#include <BenchDebug.h>
const int kLedPin = 13;

BenchDebug::BenchDebug(BenchMode mode) : gyroDrive(NULL), x25Motors(NULL)
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

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    if (gyroDrive)
    {
        delete gyroDrive;
    }
    if (x25Motors)
    {
        delete x25Motors;
    }
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleBenchInput(String command)
{
    if (command.startsWith("mo"))
    {
        if (gyroDrive)
        {
            gyroDrive->offAllAxes();
        }
        Serial.println("All motors off.");
        return true;
    }
    else if (command.startsWith("mh"))
    {
        if (gyroDrive)
        {
            gyroDrive->homeAllAxis();
        }
        if (x25Motors)
        {
            x25Motors->homeAllX25Steppers();
        }
        Serial.println("Homing all motors.");
        return true;
    }
    else if (command.startsWith("xh"))
    {
        if (x25Motors)
        {
            x25Motors->homeAllX25Steppers();
            Serial.println("Homing all X25 motors.");
        }
        return true;
    }
    else if (command.startsWith("x0"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (x25Motors)
        {
            float ratio = rString.toFloat();
            x25Motors->setPosition(0, ratio);
            Serial.println("X25 Motor 0 position set to: " + String(ratio));
        }
        return true;
    }
    else if (command.startsWith("x1"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (x25Motors)
        {
            float ratio = rString.toFloat();
            x25Motors->setPosition(1, ratio);
            Serial.println("X25 Motor 1 position set to: " + String(ratio));
        }
        return true;
    }
    else if (command.startsWith("x2"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (x25Motors)
        {
            float ratio = rString.toFloat();
            x25Motors->setPosition(2, ratio);
            Serial.println("X25 Motor 2 position set to: " + String(ratio));
        }
        return true;
    }
    else if (command.startsWith("x3"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (x25Motors)
        {
            float ratio = rString.toFloat();
            x25Motors->setPosition(3, ratio);
            Serial.println("X25 Motor 3 position set to: " + String(ratio));
        }
        return true;
    }
    else if (command.startsWith("x4"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (x25Motors)
        {
            float ratio = rString.toFloat();
            x25Motors->setPosition(4, ratio);
            Serial.println("X25 Motor 4 position set to: " + String(ratio));
        }
        return true;
    }
    else if (command.startsWith("?"))
    {
        Serial.println(F("Bench Commands:"));
        Serial.println("  --- All Stepper Motors ---");
        Serial.println("  mo - Turn off all motors");
        Serial.println("  mh - Home all motors");
        Serial.println("  -- X25 Motors ---");
        Serial.println("  xh - Home all X25 motors");
        Serial.println("  x0<ratio> - Set X25 motor 0 position (0.0 - 1.0)");
        Serial.println("  x1<ratio> - Set X25 motor 1 position (0.0 - 1.0)");
        Serial.println("  x2<ratio> - Set X25 motor 2 position (0.0 - 1.0)");
        Serial.println("  x3<ratio> - Set X25 motor 3 position (0.0 - 1.0)");
        Serial.println("  x4<ratio> - Set X25 motor 4 position (0.0 - 1.0)");
        return true;
    }
    return false;
}

void BenchDebug::handleUserInput()
{
    while (Serial.available() > 0)
    {
        char receivedChar = Serial.read(); // Einzelnes Zeichen
        if (receivedChar == 13)
        {                       // Enter erkannt
            Serial.println();   // Neue Zeile
            inputBuffer.trim(); // Eingabe bereinigen (Leerzeichen etc.)
            if (!handleBenchInput(inputBuffer))
            {
                Serial.println(F("Unknown command. Type '?' for help."));
            }
            inputBuffer = ""; // Buffer leeren
        }
        else if (receivedChar >= 32 && receivedChar <= 126)
        {
            inputBuffer += receivedChar; // Zeichen an den Buffer anhängen
            Serial.print(receivedChar);  // Eingabe zurückgeben
        }
    }
}

void BenchDebug::loop()
{
    if (millis() - heartbeat > kHeartbeatInterval)
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
    if (x25Motors)
    {
        x25Motors->updateAllX25Steppers();
    }
}