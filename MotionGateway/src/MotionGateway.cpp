#include "MotionGateway.h"
#include "DebugLog.h"

enum class RxState : uint8_t
{
  SyncBC,
  Reserved,
  Data,
  CR
};

static const size_t kMaxDataSize = 12;
static RxState state = RxState::SyncBC;
static uint8_t data[kMaxDataSize];
static uint8_t idx = 0;

MotionGateway::MotionGateway(CAN *canBus) : canBus(canBus)
{
  Serial.begin(115200);

  pinMode(kMode1Pin, INPUT_PULLUP);
  pinMode(kMode2Pin, INPUT_PULLUP);

  for (uint8_t i = 0; i < kActorNodeCount; ++i)
  {
    actorDemandMeta[i] = {0, 5000}; // maxAge of 5 seconds for resync
  }
}

MotionGateway::~MotionGateway()
{
}

void MotionGateway::loop()
{

  bool mode1PinState = digitalRead(kMode1Pin) == LOW; // Active low
  bool mode2PinState = digitalRead(kMode2Pin) == LOW; // Active low
  MotionMode newMode = MotionMode::mode0;             // Default to mode0 (Off)
  if (mode1PinState && !mode2PinState)
  {
    newMode = MotionMode::mode1; // BFF Motion Driver compatible mode
  }
  else if (!mode1PinState && mode2PinState)
  {
    newMode = MotionMode::mode2; // Sim Mode
  }
  if (newMode != mode)
  {
    DEBUGLOG_PRINTLN(String(F("Mode change: ")) + static_cast<uint8_t>(mode) + String(F(" -> ")) + static_cast<uint8_t>(newMode));

    switch (newMode)
    {
    case MotionMode::mode0:
      sendStop();
      break;
    case MotionMode::mode1:
      sendHome();
      break;
    case MotionMode::mode2:
      sendHome();
      break;
    }

    // Flash 1 time green for mode1, 2 times for mode2
    // uint8_t flashCount = (newMode == MotionMode::mode1) ? 1 : (newMode == MotionMode::mode2) ? 2
    //                                                                                          : 0;
    // digitalWrite(kStatusLedGreenPin, LOW);
    // digitalWrite(kStatusLedRedPin, LOW);
    // delay(500);

    // for (uint8_t i = 0; i < flashCount; ++i)
    // {
    //   digitalWrite(kStatusLedGreenPin, HIGH);
    //   delay(500);
    //   digitalWrite(kStatusLedGreenPin, LOW);
    //   delay(500);
    // }
    // digitalWrite(kStatusLedGreenPin, LOW);
    // digitalWrite(kStatusLedRedPin, LOW);
    // delay(500);

    mode = newMode;
  }

  // Check for maxAge resync
  checkMaxAgeResync();

  handleSerialInput();
}

void MotionGateway::handleSerialInput()
{
  if (mode == MotionMode::mode1)
  {
    while (Serial.available() > 0)
    {
      uint8_t b = (uint8_t)Serial.read();

      switch (state)
      {
      case RxState::SyncBC:
        state = (b == 0xBC) ? RxState::Reserved : RxState::SyncBC;
        break;

      case RxState::Reserved:
        state = RxState::Data;
        idx = 0;
        break;

      case RxState::Data:
        data[idx++] = b;
        if (idx >= kMaxDataSize)
        {
          state = RxState::CR; // Wait for CR after max data size
        }
        break;

      case RxState::CR:
        if (b == 0x0D) // CR
        {
          // Process complete frame
          if (idx == 12)
          {
            handleBFFFrame(data);
          }
          state = RxState::SyncBC;
        }
        break;
      }
    }
  }
  else if (mode == MotionMode::mode2)
  {
    // handleSimFrame(data);
  }
}

// Handle a complete frame of 12 data bytes (actor demands)
// BIN2B output format is - “BC” b1 b2 b3 b4 b5 b6 b7 b8 b9 b10 b11 b12 b13 0x0D (CR)
// "BC" - start of data identifier for the receiving micro controller
// byte1 - reserved

// byte2 - 8 bit binary number giving act1 demand MSB in 0-255 scale
// byte3 - 8 bit binary number giving act2 demand MSB in 0-255 scale
// byte4 - 8 bit binary number giving act3 demand MSB in 0-255 scale
// byte5 - 8 bit binary number giving act4 demand MSB in 0-255 scale
// byte6 - 8 bit binary number giving act5 demand MSB in 0-255 scale
// byte7 - 8 bit binary number giving act6 demand MSB in 0-255 scale

// byte8 - 8 bit binary number giving act1 demand LSB in 0-255 scale
// byte9 - 8 bit binary number giving act2 demand LSB in 0-255 scale
// byte10 - 8 bit binary number giving act3 demand LSB in 0-255 scale
// byte11 - 8 bit binary number giving act4 demand LSB in 0-255 scale
// byte12 - 8 bit binary number giving act5 demand LSB in 0-255 scale
// byte13 - 8 bit binary number giving act6 demand LSB in 0-255 scale

// 0x0D - single byte Carriage Return data terminator

// The 16 bit value is read by combining the MSB & LSB for each actuator, eg for Act 1 -
// Act1 16bit demand  = (b2 * 256) + b8,   in 0 to 65280 range, with 32640 mid range position
void MotionGateway::handleBFFFrame(const uint8_t *data)
{
  DEBUGLOG_PRINTLN(F("Received BFF frame"));
  uint16_t demand[kMaxDataSize / 2] = {0};
  for (uint8_t i = 0; i < kMaxDataSize / 2; ++i)
  {
    demand[i] = ((uint16_t)data[i] << 8) | data[i + kMaxDataSize / 2];
    DEBUGLOG_PRINTLN(String(F("Actuator ")) + (i + 1) + String(F(": ")) + demand[i]);
  }

  for (uint8_t i = 0; i < kActorNodeCount; ++i)
  {
    uint32_t pairDemand = (static_cast<uint32_t>(demand[i * 2]) << 16) | demand[i * 2 + 1];
    if (actorDemand[i] != pairDemand)
    {
      actorDemand[i] = pairDemand;
      // Only send actorPairDemand if system is active
      if (canBus->isSystemActive())
      {
        sendActorPairDemand(static_cast<MotionNodeId>(i + 1), demand[i * 2], demand[i * 2 + 1]);
        DEBUGLOG_PRINTLN(String(F("Sent actorPairDemand for Actor Node ")) + (i + 1) + String(F(": Act1=")) + demand[i * 2] + String(F(", Act2=")) + demand[i * 2 + 1]);
      }
    }
  }
}

void MotionGateway::handleSimFrame(const uint8_t *data)
{
}

void MotionGateway::sendActorPairDemand(MotionNodeId nodeId, uint16_t act1Demand, uint16_t act2Demand)
{
  byte data[8] = {0};

  data[0] = static_cast<uint8_t>(nodeId);
  data[1] = (act1Demand >> 8) & 0xFF; // Act1 MSB
  data[2] = act1Demand & 0xFF;        // Act1 LSB
  data[3] = (act2Demand >> 8) & 0xFF; // Act2 MSB
  data[4] = act2Demand & 0xFF;        // Act2 LSB
  // Remaining bytes can be used for additional data if needed, currently set to 0

  canBus->sendMessage(MotionMessageId::actorPairDemand, 8, data);

  // Update last send timestamp for maxAge resync
  actorDemandMeta[static_cast<uint8_t>(nodeId) - 1].lastSendTimestamp = millis();
}

void MotionGateway::checkMaxAgeResync()
{
  unsigned long now = millis();

  for (uint8_t i = 0; i < kActorNodeCount; ++i)
  {
    if (actorDemandMeta[i].lastSendTimestamp > 0 &&
        (now - actorDemandMeta[i].lastSendTimestamp) >= actorDemandMeta[i].maxAgeMs)
    {
      // Only resend if system is active
      if (canBus->isSystemActive())
      {
        DEBUGLOG_PRINTLN(String(F("MaxAge resync for Actor Node ")) + (i + 1));
        // Resend last known demand for this actor pair
        uint16_t act1Demand = (actorDemand[i] >> 16) & 0xFFFF;
        uint16_t act2Demand = actorDemand[i] & 0xFFFF;
        sendActorPairDemand(static_cast<MotionNodeId>(i + 1), act1Demand, act2Demand);
      }
    }
  }
}

void MotionGateway::sendHome()
{
  byte data[8] = {0};

  for (uint8_t i = 0; i < kActorNodeCount; ++i)
  {
    data[0] = i + 1; // nodeId
    canBus->sendMessage(MotionMessageId::actorPairHome, 8, data);
  }
}

void MotionGateway::sendStop()
{
  byte data[8] = {0};

  for (uint8_t i = 0; i < kActorNodeCount; ++i)
  {
    data[0] = i + 1; // nodeId
    canBus->sendMessage(MotionMessageId::actorPairStop, 8, data);
  }
}
