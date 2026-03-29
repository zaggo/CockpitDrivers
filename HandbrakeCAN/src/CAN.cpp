#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(Handbrake *handbrake)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      handbrake(handbrake)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
}

void CAN::loop()
{
    InstrumentCAN::loop();
    HandbrakePositionUpdate update = handbrake->getPositionUpdate();
    
    // Event-driven: send immediately when position changes
    if (update.changed) {
        sendHandbrakePosition(update.position);
        DEBUGLOG_PRINTLN(String(F("Handbrake position changed: ")) + String(update.position));
        lastPeriodicSendTime = millis(); // Reset periodic timer
    }
    
    // Periodic fallback: send position every 2 seconds
    unsigned long currentTime = millis();
    if (currentTime - lastPeriodicSendTime >= PERIODIC_SEND_INTERVAL_MS) {
        sendHandbrakePosition(handbrake->getHandbrakePosition());
        DEBUGLOG_PRINTLN(String(F("Handbrake position periodic: ")) + String(handbrake->getHandbrakePosition()));
        lastPeriodicSendTime = currentTime;
    }
}

void CAN::sendHandbrakePosition(uint8_t position)
{
    byte payload[1] = { position };
    sendMessage(static_cast<uint16_t>(CanMessageId::handbrakeStatus), 1, payload);
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: ID 0x270
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    // RXB1: Lights (0x203) und Gateway Heartbeat (0x300)
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    // Optional: weitere Filter-Slots frei lassen / duplizieren (je nach MCP2515-Lib erforderlich)
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    return true;
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
}

void CAN::onGatewayHeartbeatDiscovered()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat OK"));
}

void CAN::handleFrame(CanMessageId id, uint8_t ext, uint8_t len, const uint8_t *data)
{
        // We currently expect standard frames only (ext == 0).
    (void)ext;

    // Gateway Heartbeat is already handled by InstrumentCAN base class
    switch (id)
    {
    default:
        DEBUGLOG_PRINTLN(String(F("Handle CAN frame Id: ")) + String(static_cast<uint16_t>(id), HEX));
        break;
    }
}