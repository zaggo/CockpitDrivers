#include "DCUReceiver.h"
#include "DebugLog.h"
#include "WireEncoding.h"
#include "Heartbeat.h"

DCUReceiver::DCUReceiver(CAN *canBus) : canBus(canBus)
{
  Serial.begin(115200);
  
  // Create DCUSender instance and register it with CAN
  dcuSender = new DCUSender();
  canBus->setDCUSender(dcuSender);
  
  // Initialize message metadata with maxAge of 5 seconds
  fuelLevelMeta = {0, 5000};
  cockpitLightMeta = {0, 5000};
  transponderMeta = {0, 5000};
  rpmMeta = {0, 5000};
  odometerMeta = {0, 5000};
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

    MessageType type;
    uint8_t len;
    uint8_t payload[SerialFrameParser::kMaxPayload];
    if (frameParser.feed(b, &type, &len, payload))
    {
      handleFrame(type, len, payload);
    }
  }
}

void DCUReceiver::handleFrame(MessageType type, uint8_t len, const uint8_t *payload)
{
  // DEBUGLOG_PRINTLN(String(F("DCUReceiver::handleFrame type:")) + String(type) + String(F(" len:")) + String(len));
  switch (type)
  {
  case MessageType::SerialMessageFuel:
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

  case MessageType::SerialMessageLights:
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

  case MessageType::SerialMessageTransponder :
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

  case MessageType::SerialMessageRPM:
  {
    // Payload: float rpm (4 bytes)
    if (len != 4)
      return;

    float rpm;
    memcpy(&rpm, payload + 0, 4);

    // engine_speed_rpm can report small negative noise near idle/engine-off;
    // clamp before casting to avoid wrapping to 65535.
    if (rpm < 0.)
      rpm = 0.;

    uint16_t rpmRounded = static_cast<uint16_t>(rpm);
    if (rpmRounded != rpmValue)
    {
      rpmValue = rpmRounded;
      DEBUGLOG_PRINTLN(String(F("Received MSG_RPM Datagram rpm: ")) + String(rpmValue));
      sendRpm();
    }
    break;
  }

  case MessageType::SerialMessageOdometer:
  {
    // Payload (12 bytes): hrs1000, hrs100, hrs10, hrs1 as int8, hrsTenths, hrsHundredths as float
    if (len != 12)
      return;

    int8_t hrs1000, hrs100, hrs10, hrs1;
    float hrsTenths, hrsHundredths;
    memcpy(&hrs1000, payload + 0, 1);
    memcpy(&hrs100, payload + 1, 1);
    memcpy(&hrs10, payload + 2, 1);
    memcpy(&hrs1, payload + 3, 1);
    memcpy(&hrsTenths, payload + 4, 4);
    memcpy(&hrsHundredths, payload + 8, 4);

    uint8_t d1000 = static_cast<uint8_t>(hrs1000);
    uint8_t d100 = static_cast<uint8_t>(hrs100);
    uint8_t d10 = static_cast<uint8_t>(hrs10);
    uint8_t d1 = static_cast<uint8_t>(hrs1);
    uint8_t dTenths = static_cast<uint8_t>(hrsTenths);
    uint16_t dHundredths100 = static_cast<uint16_t>(hrsHundredths * 1000.);

    if (d1000 != tachHrs1000 || d100 != tachHrs100 || d10 != tachHrs10 ||
        d1 != tachHrs1 || dTenths != tachHrsTenths || dHundredths100 != tachHrsHundredths100)
    {
      tachHrs1000 = d1000;
      tachHrs100 = d100;
      tachHrs10 = d10;
      tachHrs1 = d1;
      tachHrsTenths = dTenths;
      tachHrsHundredths100 = dHundredths100;
      DEBUGLOG_PRINTLN(String(F("Received MSG_ODOMETER Datagram")));
      sendOdometer();
    }
    break;
  }

  default:
    // Unknown message type -> ignore
    DEBUGLOG_PRINTLN(String(F("Received unknown message type: ")) + String(static_cast<int>(type)) + String(F(" len: ")) + String(len));
    break;
  }
}

void DCUReceiver::sendFuelLevel()
{
  byte data[8] = {0};

  packBE16(data + 0, leftTankLevelKg100);
  packBE16(data + 2, rightTankLevelKg100);

  canBus->sendMessage(CanMessageId::fuelLevel, 8, data);
  
  // Update last send timestamp for maxAge resync
  fuelLevelMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendCockpitLightLevel()
{
  byte data[8] = {0};

  // Byte 0..1: Panel Dim * 1000
  packBE16(data + 0, panelDim1000);

  // Byte 2..3: Radio Dim * 1000
  packBE16(data + 2, radioDim1000);

  // Byte 4: Dome Light On/Off
  data[4] = domeLightDim1000 > 0 ? 1 : 0;

  canBus->sendMessage(CanMessageId::lights, 8, data);
  
  // Update last send timestamp for maxAge resync
  cockpitLightMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendTransponder()
{
  byte data[8] = {0};

  packBE16(data + 0, transponderCode);

  data[2] = transponderMode;
  data[3] = transponderLight;

  canBus->sendMessage(CanMessageId::transponder, 8, data);

  // Update last send timestamp for maxAge resync
  transponderMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendRpm()
{
  byte data[2] = {0};

  packBE16(data + 0, rpmValue);

  canBus->sendMessage(CanMessageId::rpm, 2, data);

  // Update last send timestamp for maxAge resync
  rpmMeta.lastSendTimestamp = millis();
}

void DCUReceiver::sendOdometer()
{
  byte data[7] = {0};

  data[0] = tachHrs1000;
  data[1] = tachHrs100;
  data[2] = tachHrs10;
  data[3] = tachHrs1;
  data[4] = tachHrsTenths;
  packBE16(data + 5, tachHrsHundredths100);

  canBus->sendMessage(CanMessageId::odometer, 7, data);

  // Update last send timestamp for maxAge resync
  odometerMeta.lastSendTimestamp = millis();
}

void DCUReceiver::checkMaxAgeResync()
{
  unsigned long now = millis();
  
  // Check fuel level message
  if (isStale(fuelLevelMeta.lastSendTimestamp, now, fuelLevelMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for fuelLevel")));
    sendFuelLevel();
  }

  // Check cockpit light level message
  if (isStale(cockpitLightMeta.lastSendTimestamp, now, cockpitLightMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for cockpitLights")));
    sendCockpitLightLevel();
  }

  // Check transponder message
  if (isStale(transponderMeta.lastSendTimestamp, now, transponderMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for transponder")));
    sendTransponder();
  }

  // Check RPM message
  if (isStale(rpmMeta.lastSendTimestamp, now, rpmMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for rpm")));
    sendRpm();
  }

  // Check odometer message
  if (isStale(odometerMeta.lastSendTimestamp, now, odometerMeta.maxAgeMs))
  {
    DEBUGLOG_PRINTLN(String(F("MaxAge resync for odometer")));
    sendOdometer();
  }
}