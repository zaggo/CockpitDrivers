#include "MotionGateway.h"
#include "DebugLog.h"

// Actor mapping tables for different modes
// Each entry defines where actor 1-6 maps to: {nodeId, motorIndex (0=m1, 1=m2)}
// Mode 1: BFF Motion Driver compatible mode
const ActorMapping MotionGateway::actorMappingMode1[6] = {
  {MotionNodeId::actorNodeId2, 1}, // actor 1 → actorPair 2, motor 2
  {MotionNodeId::actorNodeId3, 0}, // actor 2 → actorPair 3, motor 1
  {MotionNodeId::actorNodeId3, 1}, // actor 3 → actorPair 3, motor 2
  {MotionNodeId::actorNodeId1, 0}, // actor 4 → actorPair 1, motor 1
  {MotionNodeId::actorNodeId1, 1}, // actor 5 → actorPair 1, motor 2
  {MotionNodeId::actorNodeId2, 0}  // actor 6 → actorPair 2, motor 1
};

// Mode 2: SimTools
const ActorMapping MotionGateway::actorMappingMode2[6] = {
  {MotionNodeId::actorNodeId1, 1}, // actor 1 → actorPair 1, motor 2
  {MotionNodeId::actorNodeId2, 0}, // actor 2 → actorPair 2, motor 1
  {MotionNodeId::actorNodeId2, 1}, // actor 3 → actorPair 2, motor 2
  {MotionNodeId::actorNodeId3, 0}, // actor 4 → actorPair 3, motor 1
  {MotionNodeId::actorNodeId3, 1}, // actor 5 → actorPair 3, motor 2
  {MotionNodeId::actorNodeId1, 0}  // actor 6 → actorPair 1, motor 1
};

enum class RxState : uint8_t
{
  SyncB,
  SyncC,
  Reserved,
  Data,
  CR,
  GotoData,
  GotoCR
};

static const size_t kMaxDataSize = 12;
static const size_t kGotoDataSize = 14;   // 12 target bytes + duration_ms (u16 BE)
static RxState state = RxState::SyncB;
static uint8_t data[kGotoDataSize];
static uint8_t idx = 0;

MotionGateway::MotionGateway(CAN *canBus) : canBus(canBus)
{
  Serial.begin(115200);

  lastModeCheckTimestampMs = millis() - 200;
  lastDemandBatchSendTimestampMs = millis() - kDemandBatchIntervalMs;
  lastUsbHeartbeatTimestampMs = millis();

  pinMode(kMode1Pin, INPUT_PULLUP);
  pinMode(kMode2Pin, INPUT_PULLUP);
  pinMode(kArmPin, INPUT_PULLUP);

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
  const unsigned long now = millis();
  if ((now - lastModeCheckTimestampMs) >= 200)
  {
    lastModeCheckTimestampMs = now;

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

      // The parser state machine is shared between the modes - without a reset
      // the first frame after a mode change is parsed from a mid-frame offset.
      state = RxState::SyncB;
      idx = 0;
      pendingDemandValid = false;
      pendingGotoValid = false;

      mode = newMode;
    }
  }

  // Check for maxAge resync
  checkMaxAgeResync();

  if ((now - lastUsbHeartbeatTimestampMs) >= kUsbHeartbeatIntervalMs)
  {
    lastUsbHeartbeatTimestampMs = now;
    sendUsbHeartbeat();
  }

  handleSerialInput();

  // Apply the newest complete frame AFTER the serial drain: the blocking CAN
  // sends (up to ~5 ms each on TX contention) must not stall the RX loop, or
  // the serial buffer overflows and frames corrupt silently.
  if (pendingGotoValid)
  {
    pendingGotoValid = false;
    processGoto();
  }

  if (pendingDemandValid)
  {
    pendingDemandValid = false;
    processDemands(pendingDemand);
  }

  if ((now - lastStatsPrintMs) >= 5000)
  {
    lastStatsPrintMs = now;
    if (mode != MotionMode::mode0)
    {
      stats.print(canBus->txFailureCount());
      stats.reset();
    }
  }
}

void MotionGateway::handleSerialInput()
{
  const unsigned long startMs = millis();

  if (mode == MotionMode::mode1)
  {
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
    while (Serial.available() > 0 && (millis() - startMs) < kSerialProcessingBudgetMs)
    {
      uint8_t b = (uint8_t)Serial.read();

      switch (state)
      {
      case RxState::SyncB:
        if (b != 'B')
        {
          stats.resyncBytes++;
        }
        state = (b == 'B') ? RxState::SyncC : RxState::SyncB;
        break;
      case RxState::SyncC:
        if (b == 'C')      { state = RxState::Reserved; }
        else if (b == 'G') { state = RxState::GotoData; idx = 0; }
        else               { state = RxState::SyncB; }
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
          state = RxState::SyncB;
        }
        else
        {
          stats.crMissBytes++;
        }
        break;

      case RxState::GotoData:
        data[idx++] = b;
        if (idx >= kGotoDataSize)
        {
          state = RxState::GotoCR;
        }
        break;

      case RxState::GotoCR:
        if (b == 0x0D)
        {
          if (idx == kGotoDataSize)
          {
            handleGotoFrame(data);
          }
          state = RxState::SyncB;
        }
        else
        {
          stats.crMissBytes++;   // same wait-for-CR resync as the BFF CR state
        }
        break;

      default:
        state = RxState::SyncB;
        break;
      }
    }
  }
  else if (mode == MotionMode::mode2)
  {
    while (Serial.available() > 0 && (millis() - startMs) < kSerialProcessingBudgetMs)
    {
      uint8_t b = (uint8_t)Serial.read();

      switch (state)
      {
      case RxState::SyncB:
        state = (b == 'B') ? RxState::SyncC : RxState::SyncB;
        break;
      case RxState::SyncC:
        state = (b == 'C') ? RxState::Data : RxState::SyncB;
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
            handleSimToolsFrame(data);
          }
          state = RxState::SyncB;
        }
        else
        {
          stats.crMissBytes++;
        }
        break;
      default:
        state = RxState::SyncB;
        break;
      }
    }
  }
  else
  {
    // mode0: nobody parses, but the plugin may still stream - drain and drop,
    // or the 64/256-byte RX buffer overflows and the first frame after a mode
    // switch is corrupt.
    while (Serial.available() > 0 && (millis() - startMs) < kSerialProcessingBudgetMs)
    {
      Serial.read();
      stats.discardedBytes++;
    }
  }
}

// Both frame handlers only coalesce into pendingDemand ("latest wins") - the
// CAN forwarding runs once per loop() after the serial drain.
void MotionGateway::handleBFFFrame(const uint8_t *data)
{
  for (uint8_t i = 0; i < kMaxDataSize / 2; ++i)
  {
    pendingDemand[i] = ((uint16_t)data[i] << 8) | data[i + kMaxDataSize / 2];
  }
  pendingDemandValid = true;
  stats.noteFrame(millis());
}

void MotionGateway::handleSimToolsFrame(const uint8_t *data)
{
  for (uint8_t i = 0; i < kMaxDataSize / 2; ++i)
  {
    pendingDemand[i] = ((uint16_t)data[i * 2] << 8) | data[i * 2 + 1];
  }
  pendingDemandValid = true;
  stats.noteFrame(millis());
}

void MotionGateway::handleGotoFrame(const uint8_t *data)
{
  for (uint8_t i = 0; i < 6; ++i)
  {
    pendingGoto[i] = ((uint16_t)data[i] << 8) | data[i + 6];
  }
  pendingGotoDurationMs = ((uint16_t)data[12] << 8) | data[13];
  pendingGotoValid = true;
  pendingDemandValid = false;   // the goto supersedes any demand from this drain
  stats.noteFrame(millis());
}

void MotionGateway::processGoto()
{
  if (mode != MotionMode::mode1)
  {
    return;
  }

  uint16_t pairTargets[kActorNodeCount][2] = {0};
  for (uint8_t actorIdx = 0; actorIdx < 6; ++actorIdx)
  {
    const ActorMapping &map = actorMappingMode1[actorIdx];
    uint8_t pairIdx = static_cast<uint8_t>(map.nodeId) - 1;
    pairTargets[pairIdx][map.motorIndex] = pendingGoto[actorIdx];
  }

  for (uint8_t pairIdx = 0; pairIdx < kActorNodeCount; ++pairIdx)
  {
    if (!canBus->isSystemActive())
    {
      continue;
    }
    // MaxAge-resync consistency: the goto target IS the platform's demand now.
    // A resync during the move resends it as a plain demand -> actor sees
    // delta 0 and skips; the change-dedup swallows identical frames when the
    // plugin resumes streaming.
    actorDemand[pairIdx] = (static_cast<uint32_t>(pairTargets[pairIdx][0]) << 16) |
                           pairTargets[pairIdx][1];
    sendActorPairGoto(static_cast<MotionNodeId>(pairIdx + 1),
                      pairTargets[pairIdx][0], pairTargets[pairIdx][1],
                      pendingGotoDurationMs);
  }
}

void MotionGateway::sendActorPairGoto(MotionNodeId nodeId, uint16_t act1Target,
                                      uint16_t act2Target, uint16_t durationMs)
{
  byte data[8] = {0};

  data[0] = static_cast<uint8_t>(nodeId);
  data[1] = (act1Target >> 8) & 0xFF;
  data[2] = act1Target & 0xFF;
  data[3] = (act2Target >> 8) & 0xFF;
  data[4] = act2Target & 0xFF;
  data[5] = (durationMs >> 8) & 0xFF;
  data[6] = durationMs & 0xFF;

  const uint32_t sendStartUs = micros();
  canBus->sendMessage(MotionMessageId::actorPairGoto, 8, data);
  stats.noteSend(micros() - sendStartUs);

  actorDemandMeta[static_cast<uint8_t>(nodeId) - 1].lastSendTimestamp = millis();
}

void MotionGateway::processDemands(const uint16_t demand[6])
{
  const unsigned long now = millis();
  if ((now - lastDemandBatchSendTimestampMs) < kDemandBatchIntervalMs)
  {
    return;
  }
  lastDemandBatchSendTimestampMs = now;

  // Select the appropriate mapping based on current mode
  const ActorMapping *mapping = nullptr;
  switch (mode)
  {
  case MotionMode::mode1:
    mapping = actorMappingMode1;
    break;
  case MotionMode::mode2:
    mapping = actorMappingMode2;
    break;
  default:
    return; // No mapping for this mode
  }

  // Build demands for each actor pair based on the mapping
  // pairDemands[nodeId-1][motorIndex] = demand value
  uint16_t pairDemands[kActorNodeCount][2] = {0};

  for (uint8_t actorIdx = 0; actorIdx < 6; ++actorIdx)
  {
    const ActorMapping &map = mapping[actorIdx];
    uint8_t pairIdx = static_cast<uint8_t>(map.nodeId) - 1; // Convert nodeId to index (0-2)
    pairDemands[pairIdx][map.motorIndex] = demand[actorIdx];
  }

  // Send demands for each actor pair if changed
  for (uint8_t pairIdx = 0; pairIdx < kActorNodeCount; ++pairIdx)
  {
    uint32_t pairDemand = (static_cast<uint32_t>(pairDemands[pairIdx][0]) << 16) | pairDemands[pairIdx][1];

    // Only send actorPairDemand if system is active and values have changed to reduce CAN bus load
    if (actorDemand[pairIdx] != pairDemand && canBus->isSystemActive())
    {
      actorDemand[pairIdx] = pairDemand;
      sendActorPairDemand(static_cast<MotionNodeId>(pairIdx + 1),
                          pairDemands[pairIdx][0],
                          pairDemands[pairIdx][1]);
      // DEBUGLOG_PRINTLN(String(F("Sent actorPairDemand for Actor Node ")) + (pairIdx + 1) +
      //                  String(F(": Act1=")) + pairDemands[pairIdx][0] +
      //                  String(F(", Act2=")) + pairDemands[pairIdx][1]);
    }
  }
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

  const uint32_t sendStartUs = micros();
  canBus->sendMessage(MotionMessageId::actorPairDemand, 8, data);
  stats.noteSend(micros() - sendStartUs);

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

void MotionGateway::sendUsbHeartbeat()
{
  const uint8_t armed = (digitalRead(kArmPin) == LOW) ? 0x01 : 0x00;

  if (armed != lastArmedState)
  {
    lastArmedState = armed;
    DEBUGLOG_PRINTLN(String(F("Arm state: ")) + (armed ? F("ARMED") : F("DISARMED")));
  }

  const uint8_t frame[4] = {'H', 'B', armed, 0x0D};
  Serial.write(frame, sizeof(frame));
}
