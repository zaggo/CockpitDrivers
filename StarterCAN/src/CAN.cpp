#include "CAN.h"
#include "Configuration.h"
#include "DebugLog.h"

CAN::CAN(Starter *starter)
    : InstrumentCAN(kCanCSPin, kCanIntPin, CANFirmwareInfo{static_cast<uint16_t>(kNodeId), 1, 0}),
      starter(starter)
{
    DEBUGLOG_PRINTLN(F("CAN initialized"));
}

CAN::~CAN()
{
}

void CAN::loop()
{
    InstrumentCAN::loop();
    StarterPositionUpdate update = starter->getPositionUpdate();

    // Event-driven: send immediately when the switch position changes
    if (update.changed) {
        sendStarterPosition(update.position);
        DEBUGLOG_PRINTLN(String(F("Starter position changed: ")) + String(update.position));
        lastPeriodicSendTime = millis(); // Reset periodic timer
    }

    // Periodic resend: absolute state, resyncs the sim after a DCU restart
    unsigned long currentTime = millis();
    if (currentTime - lastPeriodicSendTime >= PERIODIC_SEND_INTERVAL_MS) {
        sendStarterPosition(starter->getPosition());
        DEBUGLOG_PRINTLN(String(F("Starter position periodic: ")) + String(starter->getPosition()));
        lastPeriodicSendTime = currentTime;
    }
}

void CAN::sendStarterPosition(uint8_t position)
{
    byte payload[1] = { position };
    sendMessage(static_cast<uint16_t>(CanMessageId::starterSwitch), 1, payload);
}

void CAN::onStartupFail()
{
    DEBUGLOG_PRINTLN(F("CAN startup FAIL"));
    starter->setLed(false);
}

bool CAN::instrumentBegin()
{
    // Beide RX-Buffer vergleichen alle ID-Bits
    canBus->init_Mask(0, 0, MASK_EXACT); // RXB0
    canBus->init_Mask(1, 0, MASK_EXACT); // RXB1

    // RXB0: Starter LED (0x204) und Gateway Heartbeat (0x300)
    canBus->init_Filt(0, 0, CAN_STD_ID(CanMessageId::starterLed));
    canBus->init_Filt(1, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    // RXB1: gleiche IDs nochmal (Filter-Slots muessen belegt sein)
    canBus->init_Filt(2, 0, CAN_STD_ID(CanMessageId::starterLed));
    canBus->init_Filt(3, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));
    canBus->init_Filt(4, 0, CAN_STD_ID(CanMessageId::starterLed));
    canBus->init_Filt(5, 0, CAN_STD_ID(CanMessageId::gatewayHeartbeat));

    canBus->setMode(MCP_NORMAL);

    starter->setLed(false);

    return true;
}

void CAN::onGatewayHeartbeatTimeout()
{
    DEBUGLOG_PRINTLN(F("Gateway heartbeat TIMEOUT"));
    // No live data behind a dead gateway: LED out rather than stale on.
    starter->setLed(false);
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
    case CanMessageId::starterLed:
    {
        if (len >= 1)
        {
            starter->setLed(data[0] != 0);
        }
        break;
    }
    default:
        DEBUGLOG_PRINTLN(String(F("Handle CAN frame Id: ")) + String(static_cast<uint16_t>(id), HEX));
        break;
    }
}
