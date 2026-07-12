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
    else if (command.startsWith("p"))
    {
        if (command.length() < 4)
        {
            Serial.println("Invalid format. Use p<nodeid><channel><0-100>, e.g. p11100 or p2250.");
            return true;
        }
        uint8_t nodeId = command.charAt(1) - '0';
        uint8_t channelNumber = command.charAt(2) - '0';
        if (nodeId < 1 || nodeId > 3)
        {
            Serial.println("Invalid node ID. Use 1-3 in p<nodeid><channel><0-100>.");
            return true;
        }
        if (channelNumber < 1 || channelNumber > 2)
        {
            Serial.println("Invalid channel. Use 1-2 in p<nodeid><channel><0-100>.");
            return true;
        }
        uint8_t actorNumber = static_cast<uint8_t>((nodeId - 1) * 2 + channelNumber);

        String rString = command.substring(3);
        rString.trim();
        if (rString.length() == 0)
        {
            Serial.println("Invalid format. Missing position value (0-100).");
            return true;
        }
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
        Serial.println("Position command sent to actor node " + String(nodeId) + " Channel " + String(channelNumber) + ": " + String(positionPercent) + "%"); 
        return true;
    }
    else if (command.startsWith("c"))
    {
        if (command.length() < 4)
        {
            Serial.println("Invalid format. Use c<nodeid><channel><0-100>, e.g. c11100 or c2250.");
            return true;
        }
        uint8_t nodeId = command.charAt(1) - '0';
        uint8_t channelNumber = command.charAt(2) - '0';
        if (nodeId < 1 || nodeId > 3)
        {
            Serial.println("Invalid node ID. Use 1-3 in c<nodeid><channel><0-100>.");
            return true;
        }
        if (channelNumber < 1 || channelNumber > 2)
        {
            Serial.println("Invalid channel. Use 1-2 in c<nodeid><channel><0-100>.");
            return true;
        }
        uint8_t channel = static_cast<uint8_t>(channelNumber - 1);

        String rString = command.substring(3);
        rString.trim();
        if (rString.length() == 0)
        {
            Serial.println("Invalid format. Missing position value (0-100).");
            return true;
        }
        int positionPercent = rString.toInt();
        if (positionPercent < 0 || positionPercent > 100)
        {
            Serial.println("Invalid position. Use a value between 0 and 100.");
            return true;
        }
        uint16_t actDemand = static_cast<uint16_t>((positionPercent / 100.0) * 65535); // Scale to 0-65535 for CAN message
        byte data[8] = {0};

        data[0] = static_cast<uint8_t>(nodeId);
        data[1] = channel; // Channel number in payload
        data[2] = (actDemand >> 8) & 0xFF; // Act MSB
        data[3] = actDemand & 0xFF;        // Act LSB
        // Remaining bytes can be used for additional data if needed, currently set to 0

        canBus->sendMessage(MotionMessageId::actorCalibrationMove, 8, data);
        Serial.println("Calibration Move command sent to actor node " + String(nodeId) + " Channel " + String(channelNumber) + ": " + String(positionPercent) + "%"); 
        return true;
    }  else if (command.startsWith("mi") || command.startsWith("ma"))
    {
        bool isMin = command.startsWith("mi");
        if (command.length() < 4)
        {
            Serial.println("Invalid format. Use mi<nodeid><channel> or ma<nodeid><channel>, e.g. mi11 or ma32.");
            return true;
        }
        uint8_t nodeId = command.charAt(2) - '0';
        uint8_t channelNumber = command.charAt(3) - '0';
        if (nodeId < 1 || nodeId > 3)
        {
            Serial.println("Invalid node ID. Use 1-3 in mi<nodeid><channel> / ma<nodeid><channel>.");
            return true;
        }
        if (channelNumber < 1 || channelNumber > 2)
        {
            Serial.println("Invalid channel. Use 1-2 in mi<nodeid><channel> / ma<nodeid><channel>.");
            return true;
        }
        uint8_t channel = static_cast<uint8_t>(channelNumber - 1);

        byte data[8] = {0};
        data[0] = static_cast<uint8_t>(nodeId);
        data[1] = channel; // Channel number in payload

        MotionMessageId msgId = isMin ? MotionMessageId::actorSaveLogicMin : MotionMessageId::actorSaveLogicMax;
        canBus->sendMessage(msgId, 8, data);
        Serial.println((isMin ? "Save Logical Min" : "Save Logical Max") + String(" command sent to actor node ") + String(nodeId) + " Channel " + String(channelNumber)); 
        return true;
    }
    else if (command.startsWith("?"))
    {
        Serial.println(F("Bench Commands:"));
        Serial.println("  ho<0-3> - Home Actor Node 1-3, 0 -> home all");
        Serial.println("  st<0-3> - Stop Actor Node 1-3, 0 -> stop all");
        Serial.println("  p<node><ch><0-100> - Position (node 1-3, ch 1-2), e.g. p11100 / p2250");
        Serial.println("  c<node><ch><0-100> - Calibration Move (node 1-3, ch 1-2), e.g. c11100 / c2250");
        Serial.println("  mi<node><ch> - Save current position as logical min (node 1-3, ch 1-2), e.g. mi11");
        Serial.println("  ma<node><ch> - Save current position as logical max (node 1-3, ch 1-2), e.g. ma32");
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
            if (inputBuffer.endsWith("x") || inputBuffer.endsWith("X"))
            {
                inputBuffer = inputBuffer.substring(0, inputBuffer.length() - 1);
                inputBuffer.trim();
                Serial.println(F("Input discarded."));
            }
            else if (!handleBenchInput(inputBuffer))
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