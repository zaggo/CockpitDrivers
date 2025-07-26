#include <BenchDebug.h>
const int kLedPin = 13;

BenchDebug::BenchDebug()
{
    Serial.begin(115200);
    Serial.println("Cockpit alive!");

    tachometer = new Tachometer();
    digits = new float[6];
    Serial.println("Tachometer initialized");

    startTime = millis();
    lastTime = startTime;

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    delete tachometer;
    delete[] digits;
}

void BenchDebug::loop()
{
    if (millis() - heartbeat > 1000L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    if (tachometer)
    {
        uint32_t now = millis();
        if (now - lastTime > 750L)
        {
            lastTime = now;
            float seconds = static_cast<float>(now - startTime) / 500.0 + 3600. - 20.;
            tachometer->secondsToDigits(seconds, digits);
            tachometer->displayNumber(digits);
        }
    }

    tachometer->asyncTask();
}