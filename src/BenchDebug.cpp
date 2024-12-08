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

    if (mode & kTachometer)
    {
        tachometer = new Tachometer();
        digits = new float[6];
        Serial.println("Tachometer initialized");
    }

    startTime = millis();
    lastTime = startTime;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    delete gyroDrive;
    delete x25Motors;
    delete tachometer;
    delete[] digits;
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

            if (gyroDrive)
            {
                if (inputBuffer.startsWith("o"))
                {
                    gyroDrive->offAllAxes();
                }
                else if (inputBuffer.startsWith("h"))
                {
                    gyroDrive->homeAllAxis();
                }
                else if (inputBuffer.startsWith("C"))
                {
                    gyroDrive->moveSteps(kRollTotalSteps, 0);
                }
                else if (inputBuffer.startsWith("R"))
                {
                    String rString = inputBuffer.substring(1, inputBuffer.indexOf(" "));
                    rString.trim();
                    int32_t steps = rString.toInt();
                    gyroDrive->moveSteps(steps, 0);
                }
                else if (inputBuffer.startsWith("p") || inputBuffer.startsWith("r"))
                {
                    int pTemp = pitchTargetDegrees;
                    int rTemp = rollTargetDegrees;

                    if (inputBuffer.startsWith("p"))
                    {
                        String pString = inputBuffer.substring(1, inputBuffer.indexOf(" "));
                        pString.trim();
                        pTemp = (int)((double)pString.toInt() * kAdjustmentFactor);
                    }

                    if (inputBuffer.indexOf("r") >= 0)
                    {
                        String rString = inputBuffer.substring(inputBuffer.indexOf("r") + 1);
                        rString.trim();
                        rTemp = rString.toInt();
                    }

                    gyroDrive->moveToDegree(rTemp, pTemp);
                    pitchTargetDegrees = pTemp;
                    rollTargetDegrees = rTemp;

                    Serial.print("Aktuelle Werte: pitchTargetDegrees = ");
                    Serial.print(pitchTargetDegrees);
                    Serial.print(", rollTargetDegrees = ");
                    Serial.println(rollTargetDegrees);
                }
            }
            else
            {
                Serial.println("Fehler: Falsches Eingabeformat. Bitte 's', 'p000', 'r000' oder 'p000 r000' verwenden.");
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

    if (tachometer)
    {
        uint32_t now = millis();
        if (now - lastTime > 50L)
        {
            lastTime = now;
            float seconds = static_cast<float>(now - startTime) / 1000.0 + 3600. - 20.;
            tachometer->secondsToDigits(seconds, digits);
            tachometer->displayNumber(digits);
        }
    }

    if (gyroDrive)
    {
        gyroDrive->runAllAxes();
    }
    if (x25Motors)
    {
        x25Motors->updateAllX25Steppers();
    }
}