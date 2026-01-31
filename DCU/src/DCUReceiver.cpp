#include "DCUReceiver.h"
#include "DebugLog.h"

// Message type constants
static constexpr uint8_t MSG_FUEL = 0x01;
static constexpr uint8_t MSG_LIGHTS = 0x02;
static constexpr uint8_t MSG_TRANSPONDER = 0x03;

enum class RxState : uint8_t
{
  SyncAA,
  Sync55,
  Type,
  Len,
  Payload
};

static RxState state = RxState::SyncAA;
static uint8_t type = 0;
static uint8_t len = 0;
static uint8_t buf[32];
static uint8_t idx = 0;

DCUReceiver::DCUReceiver(CAN *canBus) : canBus(canBus)
{
  Serial.begin(115200);
  
  // Create DCUSender instance and register it with CAN
  dcuSender = new DCUSender();
  canBus->setDCUSender(dcuSender);
  
  // Initialize message metadata with maxAge of 5 seconds
  fuelLevelMeta = {0, 10000};
  cockpitLightMeta = {0, 10000};
  transponderMeta = {1, 10000};
}

DCUReceiver::~DCUReceiver()
{
  delete dcuSender;
}

void DCUReceiver::loop()
{
  // Check for maxAge resync
  checkMaxAgeResync();
  
  while (Serial.available() > 0)
  {
    uint8_t b = (uint8_t)Serial.read();

    switch (state)
    {
    case RxState::SyncAA:
      state = (b == 0xAA) ? RxState::Sync55 : RxState::SyncAA;
      break;

    case RxState::Sync55:
      if (b == 0x55)
        state = RxState::Type;
      else
        state = RxState::SyncAA;
      break;

    case RxState::Type:
      type = b;
      state = RxState::Len;
      break;

    case RxState::Len:
      len = b;
      idx = 0;
      if (len > sizeof(buf))
      {
        state = RxState::SyncAA; // ungültig
      }
      else
      {
        state = RxState::Payload;
      }
      break;

    case RxState::Payload:
      buf[idx++] = b;
      if (idx >= len)
      {
        handleFrame(type, len, buf);
        state = RxState::SyncAA;
      }
      break;
    }
  }
}

void DCUReceiver::handleFrame(uint8_t type, uint8_t len, const uint8_t *payload)
{
  // DEBUGLOG_PRINTLN(String(F("DCUReceiver::handleFrame type:")) + String(type) + String(F(" len:")) + String(len));
  switch (type)
  {
  case MSG_FUEL:
  {
    // Payload: float fuelL, float fuelR (8 bytes)
    if (len != 8)
      return;

    float fuelL;
    float fuelR;
    memcpy(&fuelL, payload + 0, 4);
    memcpy(&fuelR, payload + 4, 4);

    uint16_t fuelL100 = static_cast<uint16_t>(fuelL * 100.);
    uint16_t fuelR100 = static_cast<uint16_t>(fuelR * 100.);
    if (fuelL100 != leftTankLevelKg100 || fuelR100 != rightTankLevelKg100)
    {
      leftTankLevelKg100 = fuelL100;
      rightTankLevelKg100 = fuelR100;
      DEBUGLOG_PRINTLN(String(F("Received MSG_FUEL Datagram leftTankLevelKg100: ")) + String(fuelL100) + String(F(" rightTankLevelKg100: ")) + String(fuelR100));
      sendFuelLevel();
    }
    break;
  }

  case MSG_LIGHTS:
  {
    if (len != 12)
      return;

    float panelDim;
    float radioDim;
    float domeDim;

    memcpy(&panelDim, payload + 0, 4);
    memcpy(&radioDim, payload + 4, 4);
    memcpy(&domeDim, payload + 8, 4);

    uint16_t panel1000 = static_cast<uint16_t>(panelDim * 1000.);
    uint16_t radio1000 = static_cast<uint16_t>(radioDim * 1000.);
    uint16_t dome1000 = static_cast<uint16_t>(domeDim * 1000.);

    // Cache values (add these members if not present yet)
    if (panel1000 != panelDim1000 || radio1000 != radioDim1000 || dome1000 != domeLightDim1000)
    {
      panelDim1000 = panel1000;
      radioDim1000 = radio1000;
      domeLightDim1000 = dome1000;
      DEBUGLOG_PRINTLN(String(F("Received MSG_LIGHTS Datagram panel1000: ")) + String(panel1000) + String(F(" radio1000: ")) + String(radio1000) + String(F(" dome1000: ")) + String(dome1000));
      sendCockpitLightLevel();
    }
    break;
  }

  case MSG_TRANSPONDER:
  {
    if (len != 4)
      return;

    uint16_t code;
    uint8_t mode;
    uint8_t light;

    memcpy(&code, payload + 0, 2);
    memcpy(&mode, payload + 2, 1);
    memcpy(&light, payload + 3, 1);

    if (code != transponderCode || mode != transponderMode || light != transponderLight)
    {
      transponderCode = code;
      transponderMode = mode;
      transponderLight = light;
      DEBUGLOG_PRINTLN(String(F("Received MSG_TRANSPONDER Datagram code: ")) + String(code) + String(F(" mode: ")) + String(mode) + String(F(" light: ")) + String(light));
      sendTransponder();
    }
    break;
  }
  default:
    // Unknown message type -> ignore
    break;
  }
}

void DCUReceiver::sendFuelLevel()
{
  byte data[8] = {0};

  data[0] = static_cast<uint8_t>((leftTankLevelKg100 >> 8) & 0xff);
  data[1] = static_cast<uint8_t>(leftTankLevelKg100 & 0xff);

  data[2] = static_cast<uint8_t>((rightTankLevelKg100 >> 8) & 0xff);
  data[3] = static_cast<uint8_t>(rightTankLevelKg100 & 0xff);

  canBus->sendMessage(CanMessageId::fuelLevel, 8, data);
  
  // Update last send timestamp for maxAge resync
  fuelLevelMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendCockpitLightLevel()
{
  byte data[8] = {0};

  // Byte 0..1: Panel Dim * 1000
  data[0] = (uint8_t)((panelDim1000 >> 8) & 0xFF);
  data[1] = (uint8_t)(panelDim1000 & 0xFF);

  // Byte 2..3: Radio Dim * 1000
  data[2] = (uint8_t)((radioDim1000 >> 8) & 0xFF);
  data[3] = (uint8_t)(radioDim1000 & 0xFF);

  // Byte 4: Dome Light On/Off
  data[4] = (uint8_t)((domeLightDim1000 >> 8) & 0xFF);
  data[5] = (uint8_t)(domeLightDim1000 & 0xFF);

  canBus->sendMessage(CanMessageId::lights, 8, data);
  
  // Update last send timestamp for maxAge resync
  cockpitLightMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendTransponder()
{
  byte data[8] = {0};

  data[0] = static_cast<uint8_t>((transponderCode >> 8) & 0xff);
  data[1] = static_cast<uint8_t>(transponderCode & 0xff);

  data[2] = transponderMode;
  data[3] = transponderLight;

  canBus->sendMessage(CanMessageId::transponder, 8, data);
  
  // Update last send timestamp for maxAge resync
  transponderMeta.lastSendTimestamp = millis();
}

bool DCUReceiver::readBytes(uint8_t *dst, size_t n)
{
  size_t got = 0;
  while (got < n)
  {
    int c = Serial.read();
    if (c < 0)
      return false;
    dst[got++] = (uint8_t)c;
  }
  return true;
}

void DCUReceiver::checkMaxAgeResync()
{
  unsigned long now = millis();
  
  // Check fuel level message
  if (fuelLevelMeta.lastSendTimestamp > 0 && 
      (now - fuelLevelMeta.lastSendTimestamp) >= fuelLevelMeta.maxAgeMs)
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for fuelLevel")));
    sendFuelLevel();
  }
  
  // Check cockpit light level message
  if (cockpitLightMeta.lastSendTimestamp > 0 && 
      (now - cockpitLightMeta.lastSendTimestamp) >= cockpitLightMeta.maxAgeMs)
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for cockpitLights")));
    sendCockpitLightLevel();
  }

  // Check transponder message
  if (transponderMeta.lastSendTimestamp > 0 && 
      (now - transponderMeta.lastSendTimestamp) >= transponderMeta.maxAgeMs)
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for transponder")));
    sendTransponder();
  }
} 