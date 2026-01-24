#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

volatile bool CAN::canIrq = false;
CAN *CAN::instance = nullptr;

CAN::CAN(FuelGauge *fuelGauge) : fuelGauge(fuelGauge)
{
    canBus = new MCP_CAN(kCanCSPin);

    // MCP2515 /INT is open-drain, active-low. Use pull-up.
    pinMode(kCanIntPin, INPUT_PULLUP);

    // Hook interrupt to wake our loop when frames arrive.
    // NOTE: class ISR needs a static entry point; we keep a single active instance.
    instance = this;
    canIrq = false;
    attachInterrupt(digitalPinToInterrupt(kCanIntPin), CAN::onCanInterrupt, FALLING);

    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
    detachInterrupt(digitalPinToInterrupt(kCanIntPin));
    instance = nullptr;
    delete canBus;
}

bool CAN::begin()
{
    if (canBus->begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) != CAN_OK)
    {
        DEBUGLOG_PRINTLN(F("CAN init fail"));
        return false;
    }

    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: ID 0x270
    canBus->init_Filt(0, 0, CAN_STD_ID(CanStateId::fuelLevel));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanStateId::fuelLevel)); // zweiter Filter optional identisch

    // RXB1: Lights (0x203) und Gateway Heartbeat (0x300)
    canBus->init_Filt(2, 0, CAN_STD_ID(CanStateId::lights));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanStateId::gatewayHeartbeat));
    // Optional: weitere Filter-Slots frei lassen / duplizieren (je nach MCP2515-Lib erforderlich)
    canBus->init_Filt(4, 0, CAN_STD_ID(CanStateId::lights));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanStateId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    fuelGauge->moveServo(leftTank, 0);
    fuelGauge->moveServo(rightTank, 0);
    fuelGauge->setBrightness(0);

    isStarted = true;
    return true;
}

void CAN::onCanInterrupt()
{
    // Keep ISR tiny: no SPI, no Serial.
    canIrq = true;
}

void CAN::loop()
{    
    if (!isStarted) {
        return;
    }

    // --- Heartbeat TX (Instrument -> DCU), 2 Hz mit kleinem Offset pro Node ---
    const uint32_t now = millis();
    const uint32_t periodMs = 500;
    const uint32_t offsetMs = (uint32_t)kNodeId * 20; // vermeidet gleichzeitige HBs

    if (lastInstrumentHeartbeatSendMs == 0) {
        lastInstrumentHeartbeatSendMs = now + offsetMs; // erster Sendetermin
    }

    if ((int32_t)(now - lastInstrumentHeartbeatSendMs) >= 0) {
        sendInstrumentHeartbeat();
        lastInstrumentHeartbeatSendMs += periodMs;
    }

    // --- Gateway Heartbeat Timeout (DCU -> Instrument) ---
    const uint32_t timeoutMs = 1500;
    const bool alive = (lastGatewayHeartbeatMs != 0) && (now - lastGatewayHeartbeatMs <= timeoutMs);
    if (alive != gatewayAlive) {
        gatewayAlive = alive;
        DEBUGLOG_PRINTLN(gatewayAlive ? F("Gateway heartbeat OK") : F("Gateway heartbeat TIMEOUT"));
        // Optional: Failsafe. Für FuelGauge z.B. Helligkeit runter.
        if (!gatewayAlive) {
            fuelGauge->setBrightness(0);
        } else {
            fuelGauge->setBrightness(255);
        }
    }

    // Fast path: no interrupt seen and line is high -> nothing to do.
    if (!canIrq && digitalRead(kCanIntPin) == HIGH)
    {
        return;
    }

    // Clear flag early; if more frames arrive while draining, ISR will set it again.
    noInterrupts();
    canIrq = false;
    interrupts();

    // Drain all pending frames. INT stays low while RX buffers contain unread frames.
    while (digitalRead(kCanIntPin) == LOW)
    {
        if (canBus->checkReceive() != CAN_MSGAVAIL)
        {
            // Sometimes INT can lag a tiny bit; break to avoid busy-loop.
            break;
        }

        unsigned long rxId = 0;
        byte ext = 0;
        byte len = 0;
        byte buf[8] = {0};

        canBus->readMsgBuf(&rxId, &ext, &len, buf);
        handleFrame(rxId, ext, len, buf);
    }
}

void CAN::handleFrame(uint32_t id, uint8_t ext, uint8_t len, const uint8_t *data)
{
    DEBUGLOG_PRINTLN(String(F("CAN Message received: ID ")) + id);

    // We currently expect standard frames only (ext == 0).
    (void)ext;

    // IDs from mcp_can are the actual 11-bit ID (e.g. 0x270), even though filters use (ID<<16).
    switch (static_cast<CanStateId>(id))
    {
    case CanStateId::fuelLevel:
    {
        if (len >= 8)
        {
            const uint16_t fuelLeftKg100 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            const uint16_t fuelRightKg100 = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);

            fuelGauge->moveServo(leftTank, static_cast<float>(fuelLeftKg100) / 100.);
            fuelGauge->moveServo(rightTank, static_cast<float>(fuelRightKg100) / 100.);
        }
        break;
    }

    case CanStateId::lights:
    {
        if (len >= 8)
        {
            const uint16_t panelDim1000 = (static_cast<uint16_t>(data[0]) << 8) | static_cast<uint16_t>(data[1]);
            // Not in this instrument:
            // const uint16_t radioDim1000 = (static_cast<uint16_t>(data[2]) << 8) | static_cast<uint16_t>(data[3]);
            // const uint16_t domeLightDim1000 = (static_cast<uint16_t>(data[4]) << 8) | static_cast<uint16_t>(data[5]);
            float ratio = constrain(static_cast<float>(panelDim1000) / 1000., 0., 1.);
            uint8_t pwm = static_cast<uint8_t>(ratio * 255.);
            fuelGauge->setBrightness(pwm);
        }
        break;
    }

    case CanStateId::gatewayHeartbeat:
    {
        updateGatewayHeartbeat(len, data);
        break;
    }

    default:
        break;
    }
}

void CAN::sendMessage(CanStateId id, uint8_t len, byte* data)
{
  // send data:  ID = 0x100, Standard CAN Frame, Data length = 8 bytes, 'data' = array of data bytes to send
  uint8_t sndStat = canBus->sendMsgBuf(static_cast<unsigned long>(id), 0, len, data);
  if (sndStat == CAN_OK)
  {
    //DEBUGLOG_PRINTLN(String(F("Message Sent Successfully to id 0x"))+String(static_cast<unsigned long>(id), HEX));
  }
  else
  {
    DEBUGLOG_PRINTLN(String(F("Error Sending Message:"))+sndStat);
  }
}

void CAN::sendInstrumentHeartbeat() {
    // CAN ID 0x301 (instrumentHeartbeat), payload 8 bytes:
    // [0]=nodeId, [1]=fwMajor, [2]=fwMinor, [3]=flags, [4..7]=uptime/10ms (u32, big endian)
    byte data[8] = {0};
    data[0] = kNodeId;
    data[1] = kFwMajor;
    data[2] = kFwMinor;
    data[3] = gatewayAlive ? 0x01 : 0x00; // bit0=OK (optional)

    const uint32_t uptime10 = millis() / 10;
    data[4] = (uint8_t)((uptime10 >> 24) & 0xFF);
    data[5] = (uint8_t)((uptime10 >> 16) & 0xFF);
    data[6] = (uint8_t)((uptime10 >> 8) & 0xFF);
    data[7] = (uint8_t)(uptime10 & 0xFF);

    sendMessage(CanStateId::instrumentHeartbeat, 8, data);
}

void CAN::updateGatewayHeartbeat(uint8_t len, const uint8_t* data) {
    if (len < 8) return;

    // Validate nodeId for gateway (expected 0). If you ever change it, adjust here.
    const uint8_t nodeId = data[0];
    if (nodeId != 0) return;

    lastGatewayHeartbeatMs = millis();

    // Optional: you could parse version/flags/uptime here if needed.
}