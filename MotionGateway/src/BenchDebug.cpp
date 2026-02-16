#include <BenchDebug.h>
const int kLedPin = 13;

BenchDebug::BenchDebug(CAN* canBus) : canBus(canBus)
{
    #if !DEBUGLOG_ENABLE
    Serial.begin(115200);
    #endif
    Serial.println("MotionGatewayBenchTest alive!");

    inputBuffer = "";
    pinMode(kLedPin, OUTPUT);
    digitalWrite(kLedPin, heartbeatLedOn);
    Serial.println("System running...");
}

BenchDebug::~BenchDebug()
{
}

const int kMaxCommandLength = 10;
bool BenchDebug::handleBenchInput(String command)
{
    if (command.startsWith("ho"))
    {
        String rString = command.substring(2);
        rString.trim();
        int nodeId = rString.toInt();
        byte data[8] = {0};
        if (nodeId == 0)
        {
            for (uint8_t i = 1; i <= 3; ++i)
            {
                data[0] = i; // Actor Node ID in payload
                canBus->sendMessage(MotionMessageId::actorPairHome, 8, data);
            }
            Serial.println("Home command sent to all actors.");
        }
        else if (nodeId >= 1 && nodeId <= 3)
        {
            data[0] = nodeId; // Actor Node ID in payload
            canBus->sendMessage(MotionMessageId::actorPairHome, 8, data);
            Serial.println("Home command sent to actor node " + String(nodeId) + ".");
        }
        return true;
    }
    else if (command.startsWith("st"))
    {
        String rString = command.substring(2);
        rString.trim();
        int nodeId = rString.toInt();
        byte data[8] = {0};
        if (nodeId == 0)
        {
            for (uint8_t i = 1; i <= 3; ++i)
            {
                data[0] = i; // Actor Node ID in payload
                canBus->sendMessage(MotionMessageId::actorPairStop, 8, data);
            }
            Serial.println("Stop command sent to all actors.");
        }
        else if (nodeId >= 1 && nodeId <= 3)
        {
            data[0] = nodeId; // Actor Node ID in payload
            canBus->sendMessage(MotionMessageId::actorPairStop, 8, data);
            Serial.println("Stop command sent to actor node " + String(nodeId) + ".");
        }
        return true;
    }
    else if (command.startsWith("p1") || command.startsWith("p2") || command.startsWith("p3") || command.startsWith("p4") || command.startsWith("p5") || command.startsWith("p6"))
    {
        uint8_t actorNumber = command.charAt(1) - '0'; // Extract node number from command
        if (actorNumber < 1 || actorNumber > 6)
        {
            Serial.println("Invalid node ID. Use p1, p2, p3, p4, p5, or p6.");
            return true;
        }
        uint8_t nodeId = (actorNumber + 1) / 2; // Map p1/p2 to node 1, p3/p4 to node 2, p5/p6 to node 3

        String rString = command.substring(2);
        rString.trim();
        int positionPercent = rString.toInt();
        if (positionPercent < 0 || positionPercent > 100)        {
            Serial.println("Invalid position. Use a value between 0 and 100.");
            return true;
        }
        uint16_t actDemand = static_cast<uint16_t>((positionPercent / 100.0) * 65535); // Scale to 0-65535 for CAN message
        actorDemand[actorNumber - 1] = actDemand; // Store the demand in the array
        byte data[8] = {0};

        data[0] = static_cast<uint8_t>(nodeId);
        data[1] = (actorDemand[nodeId*2 - 2] >> 8) & 0xFF; // Act1 MSB
        data[2] = actorDemand[nodeId*2 - 2] & 0xFF;        // Act1 LSB
        data[3] = (actorDemand[nodeId*2 - 1] >> 8) & 0xFF; // Act2 MSB
        data[4] = actorDemand[nodeId*2 - 1] & 0xFF;        // Act2 LSB
        // Remaining bytes can be used for additional data if needed, currently set to 0

        canBus->sendMessage(MotionMessageId::actorPairDemand, 8, data);
        Serial.println("Position command sent to actor node " + String(nodeId) + " M" + String(actorNumber) + ": " + String(positionPercent) + "%"); 
        return true;
    }
    else if (command.startsWith("?"))
    {
        Serial.println(F("Bench Commands:"));
        Serial.println("  ho<0-3> - Home Actor Node 1-3, 0 -> home all");
        Serial.println("  st<0-3> - Stop Actor Node 1-3, 0 -> stop all");
        Serial.println("  p1<0-100> - Position Actor Node 1 M1 (0-100%)");
        Serial.println("  p2<0-100> - Position Actor Node 1 M2 (0-100%)");
        Serial.println("  p3<0-100> - Position Actor Node 2 M1 (0-100%)");
        Serial.println("  p4<0-100> - Position Actor Node 2 M2 (0-100%)");
        Serial.println("  p5<0-100> - Position Actor Node 3 M1 (0-100%)");
        Serial.println("  p6<0-100> - Position Actor Node 3 M2 (0-100%)");
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
        else if(receivedChar >= 32 && receivedChar <= 126)
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
}