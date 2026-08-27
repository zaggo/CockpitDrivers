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
    else if (command.startsWith("tr") || command.startsWith("ta") || command.startsWith("td"))
    {
        return handleTestCommand(command);
    }
    else if (command.startsWith("gt"))
    {
        // gt <node 1-3|0=all> <act1 %> <act2 %> <duration ms> - profiled goto move
        int node = 0, p1 = 0, p2 = 0, dur = 0;
        if (sscanf(command.c_str() + 2, "%d %d %d %d", &node, &p1, &p2, &dur) != 4 ||
            node < 0 || node > 3 || p1 < 0 || p1 > 100 || p2 < 0 || p2 > 100 ||
            dur < 100 || dur > 30000)
        {
            Serial.println("Usage: gt <node 1-3|0=all> <act1 0-100> <act2 0-100> <duration 100-30000 ms>");
            return true;
        }
        const uint16_t d1 = static_cast<uint16_t>((p1 / 100.0) * 65280);
        const uint16_t d2 = static_cast<uint16_t>((p2 / 100.0) * 65280);
        byte data[8] = {0};
        data[1] = (d1 >> 8) & 0xFF;
        data[2] = d1 & 0xFF;
        data[3] = (d2 >> 8) & 0xFF;
        data[4] = d2 & 0xFF;
        data[5] = (static_cast<uint16_t>(dur) >> 8) & 0xFF;
        data[6] = static_cast<uint16_t>(dur) & 0xFF;
        for (uint8_t n = 1; n <= 3; ++n)
        {
            if (node != 0 && node != n) continue;
            data[0] = n;
            canBus->sendMessage(MotionMessageId::actorPairGoto, 8, data);
            Serial.println("Goto sent to node " + String(n) + ": " + String(p1) + "%/" +
                           String(p2) + "% in " + String(dur) + " ms");
        }
        return true;
    }
    else if (command.startsWith("gs"))
    {
        return handleStreamCommand(command);
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
        Serial.println("  gt <node|0> <a1%> <a2%> <ms> - profiled goto move (arm/disarm path)");
        Serial.println(F("Test bench (actor testbench firmware required):"));
        Serial.println("  tr <node> <strat> <rate> <amp%> <chmask> <smode> <durS> [param] - start test run");
        Serial.println("     strat: 0 single-move 1 current-algo 2 exact-speed 3 stream+current 4 stream+exact 9 passthrough");
        Serial.println("     smode: 0 none 1 getP-series 2 timing-series 3 getP every param-th cycle");
        Serial.println("  ta <node> - abort test run");
        Serial.println("  td <node> - dump samples (prints TH/TD/TS CSV lines)");
        Serial.println("  gs <node> <wave:0 tri|1 sine> <rateHz> <amp%> <durS> - 0x110 demand stream, gs 0 stops");
        return true;
    }
    return false;
}

// Parses up to maxCount whitespace-separated integers following the two-letter
// command. Returns the number of values parsed.
static uint8_t parseArgs(const String &command, long *values, uint8_t maxCount)
{
    uint8_t count = 0;
    int idx = 2; // skip the two-letter command
    const int len = command.length();
    while (count < maxCount)
    {
        while (idx < len && command.charAt(idx) == ' ')
        {
            ++idx;
        }
        if (idx >= len)
        {
            break;
        }
        int end = idx;
        while (end < len && command.charAt(end) != ' ')
        {
            ++end;
        }
        values[count++] = command.substring(idx, end).toInt();
        idx = end;
    }
    return count;
}

bool BenchDebug::handleTestCommand(const String &command)
{
    long args[8] = {0};
    const uint8_t argCount = parseArgs(command, args, 8);
    if (argCount < 1 || args[0] < 1 || args[0] > 3)
    {
        Serial.println(F("Invalid node. Use tr/ta/td <node 1-3> ..."));
        return true;
    }
    const uint8_t nodeId = (uint8_t)args[0];
    byte data[8] = {0};
    data[0] = nodeId;

    if (command.startsWith("ta"))
    {
        canBus->sendMessage(MotionMessageId::actorTestAbort, 8, data);
        Serial.println("Test abort sent to actor node " + String(nodeId) + ".");
        return true;
    }
    if (command.startsWith("td"))
    {
        canBus->sendMessage(MotionMessageId::actorTestDumpRequest, 8, data);
        Serial.println("Dump request sent to actor node " + String(nodeId) + ".");
        return true;
    }

    // tr <node> <strat> <rate> <amp%> <chmask> <smode> <durS> [param]
    if (argCount < 7)
    {
        Serial.println(F("Use: tr <node> <strat> <rate> <amp%> <chmask> <smode> <durS> [param]"));
        return true;
    }
    data[1] = (uint8_t)args[1]; // strategy
    data[2] = (uint8_t)args[2]; // rateHz
    data[3] = (uint8_t)args[3]; // amplitudePct
    data[4] = (uint8_t)args[4]; // channelMask
    data[5] = (uint8_t)args[5]; // sampleMode
    data[6] = (uint8_t)args[6]; // durationSec
    data[7] = (uint8_t)(argCount >= 8 ? args[7] : 1);
    canBus->sendMessage(MotionMessageId::actorTestStart, 8, data);
    Serial.println("Test start sent to actor node " + String(nodeId) +
                   " strat=" + String(data[1]) + " rate=" + String(data[2]) +
                   " amp=" + String(data[3]) + "% chmask=" + String(data[4]) +
                   " smode=" + String(data[5]) + " dur=" + String(data[6]) +
                   "s param=" + String(data[7]));
    return true;
}

bool BenchDebug::handleStreamCommand(const String &command)
{
    long args[5] = {0};
    const uint8_t argCount = parseArgs(command, args, 5);
    if (argCount >= 1 && args[0] == 0)
    {
        streamActive = false;
        Serial.println(F("Stream stopped."));
        return true;
    }
    if (argCount < 5 || args[0] < 1 || args[0] > 3 || args[2] < 1 || args[2] > 100 ||
        args[3] < 1 || args[3] > 100 || args[4] < 1)
    {
        Serial.println(F("Use: gs <node 1-3> <wave:0 tri|1 sine> <rateHz 1-100> <amp% 1-100> <durS>, gs 0 stops"));
        return true;
    }

    streamNodeId = (uint8_t)args[0];
    streamWave = (uint8_t)(args[1] != 0);
    streamAmpPct = (uint8_t)args[3];
    streamPeriodUs = 1000000UL / (uint32_t)args[2];
    streamStartMs = millis();
    streamEndMs = streamStartMs + (uint32_t)args[4] * 1000UL;
    streamNextTickUs = micros();
    streamFramesSent = 0;
    streamMissedTicks = 0;
    streamActive = true;
    Serial.println("Stream started: node " + String(streamNodeId) +
                   (streamWave ? " sine" : " triangle") + " @" + String(args[2]) +
                   "Hz amp=" + String(streamAmpPct) + "% dur=" + String(args[4]) + "s");
    return true;
}

void BenchDebug::sendPairDemand(uint8_t nodeId, uint16_t demand)
{
    actorDemand[nodeId * 2 - 2] = demand;
    actorDemand[nodeId * 2 - 1] = demand;
    byte data[8] = {0};
    data[0] = nodeId;
    data[1] = (demand >> 8) & 0xFF;
    data[2] = demand & 0xFF;
    data[3] = (demand >> 8) & 0xFF;
    data[4] = demand & 0xFF;
    canBus->sendMessage(MotionMessageId::actorPairDemand, 8, data);
}

void BenchDebug::tickStreamGenerator()
{
    if (!streamActive)
    {
        return;
    }

    const uint32_t nowMs = millis();
    if ((int32_t)(nowMs - streamEndMs) >= 0)
    {
        streamActive = false;
        Serial.println("Stream done: " + String(streamFramesSent) + " frames, " +
                       String(streamMissedTicks) + " missed ticks.");
        return;
    }

    const uint32_t nowUs = micros();
    if ((int32_t)(nowUs - streamNextTickUs) < 0)
    {
        return;
    }
    streamNextTickUs += streamPeriodUs;
    if ((int32_t)(nowUs - streamNextTickUs) >= 0)
    {
        streamMissedTicks += (nowUs - streamNextTickUs) / streamPeriodUs + 1;
        streamNextTickUs = nowUs + streamPeriodUs;
    }

    // Same shape as the actor bench: full peak-to-peak traversal in 3 s per leg
    // (6 s wave period), amplitude centered at mid-scale.
    const uint32_t kWavePeriodMs = 6000;
    const uint32_t phaseMs = (nowMs - streamStartMs) % kWavePeriodMs;
    const int32_t halfAmp = (int32_t)(((uint32_t)65535 * streamAmpPct) / 200);

    int32_t offset;
    if (streamWave == 0)
    {
        // Triangle: -half .. +half .. -half
        if (phaseMs < kWavePeriodMs / 2)
        {
            offset = -halfAmp + (int32_t)(((int64_t)2 * halfAmp * phaseMs) / (kWavePeriodMs / 2));
        }
        else
        {
            const uint32_t p2 = phaseMs - kWavePeriodMs / 2;
            offset = halfAmp - (int32_t)(((int64_t)2 * halfAmp * p2) / (kWavePeriodMs / 2));
        }
    }
    else
    {
        const float phase = (2.0f * PI * (float)phaseMs) / (float)kWavePeriodMs;
        offset = (int32_t)((float)halfAmp * sinf(phase));
    }

    int32_t demand = 32768L + offset;
    if (demand < 0) demand = 0;
    if (demand > 65535L) demand = 65535L;
    sendPairDemand(streamNodeId, (uint16_t)demand);
    ++streamFramesSent;
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

    tickStreamGenerator();
    handleUserInput();
}