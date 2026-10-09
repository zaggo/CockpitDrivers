#include <BenchDebug.h>
const int kLedPin = 13;

BenchDebug::BenchDebug()
{
    Serial.begin(115200);
    Serial.println("Cockpit alive!");

    inputBuffer = "";

    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);

    servos = new Servos();

    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
    delete servos;
}


const int kMaxCommandLength = 10;

 // Available commands:
 //     ?          Show this help message
 //     s <id> <float: 0...1>  Set servo position
 //     o          Set all servos to position 0
 //     w <id>     Sweep servo from 0 to 1
void BenchDebug::handleUserInput()
{
    if (Serial.available() > 0)
    {
        char c = Serial.read();
        Serial.print(c); // Echo entered character

        if (c == '\n' || c == '\r')
        {
            if (inputBuffer.length() > 0)
            {
                if (inputBuffer.startsWith("?"))
                {
                    Serial.println("Available commands:");
                    Serial.println("  ?          Show this help message");
                    Serial.println("  s <id> <float: 0...1>  Set servo position");
                    Serial.println("  o          Set all servos to position 0");
                    Serial.println("  w <id>     Sweep servo from 0 to 1");
                }
                else if (inputBuffer.startsWith("s "))
                {
                    int spaceIndex = inputBuffer.indexOf(' ', 2);
                    if (spaceIndex != -1)
                    {
                        String idStr = inputBuffer.substring(2, spaceIndex);
                        String posStr = inputBuffer.substring(spaceIndex + 1);
                        int servoId = idStr.toInt();
                        float position = posStr.toFloat();

                        // Validate servoId and position
                        if (servoId < 0 || servoId >= kServoCount)
                        {
                            Serial.println("Error: Invalid servo ID.");
                        }
                        else if (position < 0.0f || position > 1.0f)
                        {
                            Serial.println("Error: Position must be between 0 and 1.");
                        }
                        else
                        {
                            servos->setPosition(servoId, position);
                            Serial.print("Set servo ");
                            Serial.print(servoId);
                            Serial.print(" to position ");
                            Serial.println(position, 3);
                        }
                    }
                    else
                    {
                        Serial.println("Error: Invalid command format. Usage: s <id> <float: 0...1>");
                    }
                }
                else if (inputBuffer == "o")
                {
                    float positions[kServoCount] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
                    servos->setPositions(kServoCount, positions);
                    Serial.println("Set all servos to position 0");
                }
                else if (inputBuffer.startsWith("w "))
                {
                    String idStr = inputBuffer.substring(2);
                    int servoId = idStr.toInt();
                    if (servoId < 0 || servoId >= kServoCount)
                    {
                        Serial.println("Error: Invalid servo ID.");
                    }
                    else
                    {
                        Serial.print("Sweeping servo ");
                        Serial.println(servoId);
                        servos->setPosition(servoId, 0.0f);
                        delay(500); // Wait for servo to reach position
                        for (int sweep = 0; sweep < 3; ++sweep)
                        {
                            for (float pos = 0.0f; pos <= 1.0f; pos += 0.05f)
                            {
                                servos->setPosition(servoId, pos);
                                delay(20);
                            }
                            for (float pos = 1.0f; pos >= 0.0f; pos -= 0.05f)
                            {
                                servos->setPosition(servoId, pos);
                                delay(20);
                            }
                        }
                        Serial.println("Sweep complete.");
                    }
                }
            }
            inputBuffer = "";
        }
        else
        {
            if (inputBuffer.length() < kMaxCommandLength)
            {
                inputBuffer += c;
            }
        }
    }
}


void BenchDebug::loop()
{
    if (millis() - heartbeat > 600L)
    {
        heartbeat = millis();
        digitalWrite(kLedPin, heartbeatLedOn ? HIGH : LOW);
        heartbeatLedOn = !heartbeatLedOn;
    }

    handleUserInput();
}