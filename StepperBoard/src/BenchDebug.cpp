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

    if (mode & kTransponder)
    {
        transponder = new Transponder();
        Serial.println("Transponder initialized");
    }

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    delete gyroDrive;
    delete x25Motors;
    delete transponder;
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
    else if (command.startsWith("ti"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (transponder)
        {
            bool ident = (rString.toInt() != 0);
            transponder->setIdent(ident);
            Serial.println("Transponder Ident set to: " + String(ident ? "ON" : "OFF"));
        }
        return true;
    }
    else if (command.startsWith("ta"))
    {
        if (transponder)
        {
            transponder->setMode(Transponder::alt);
            Serial.println("Transponder mode set to ALT.");
        }
        return true;
    }
    else if (command.startsWith("ts"))
    {
        if (transponder)
        {
            transponder->setMode(Transponder::stdby);
            Serial.println("Transponder mode set to STANDBY.");
        }
        return true;
    }
    else if (command.startsWith("t1"))
    {
        if (transponder)
        {
            transponder->setMode(Transponder::on);      
            Serial.println("Transponder mode set to ON.");
        }
        return true;
    }
    else if (command.startsWith("t0"))
    {
        if (transponder)
        {
            transponder->setMode(Transponder::off);
            Serial.println("Transponder mode set to OFF.");
        }
        return true;
    }
    else if (command.startsWith("tb"))
    {
        String rString = command.substring(2);
        rString.trim();
        if (transponder)
        {
            int buttonIndex = rString.toInt();
            if (buttonIndex >= 0 && buttonIndex <= 9)
            {
                // Simulate button press    
                transponder->pressNumberButton(buttonIndex);
                Serial.println("Transponder button " + String(buttonIndex) + " pressed.");
            }
        }
        return true;
    }
    else if (command.startsWith("?"))
    {
        Serial.println(F("Bench Commands:"));
        Serial.println("  --- All Stepper Motors ---");
        Serial.println("  mo - Turn off all motors");
        Serial.println("  mh - Home all motors");
        Serial.println("  --- Transponder ---");
        Serial.println("  ti<0|1> - Set Ident off/on");
        Serial.println("  ta - Set Transponder to Alt mode");
        Serial.println("  ts - Set Transponder to Standby mode");
        Serial.println("  t1 - Set Transponder to On mode");
        Serial.println("  t0 - Set Transponder to Off mode");
        Serial.println("  tb<0-9> - Press Transponder button 0-9");
        return true;
    }
    return false;
}

void BenchDebug::handleUserInput()
{
    static String inputBuffer = ""; // Zwischenspeicher für serielle Eingaben

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
        else if(receivedChar >= 32 && receivedChar <= 126)
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
    if (transponder)
    {
        transponder->tick();
    }
}