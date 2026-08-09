#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"
#include <SerialMessageId.h>

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
#include "Transponder.h"
CAN *canBus;
Transponder *transponder;
#endif

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Transponder initializing..."));

  #if BENCHDEBUG
  benchDebug = new BenchDebug();
  #else
  transponder = new Transponder();
  canBus = new CAN(transponder);
  if (!canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Transponder started up!"));
  } else {
    DEBUGLOG_PRINTLN(F("Transponder FAILED to start!"));
  }
  #endif
}

#if !BENCHDEBUG
void sendChangesToDCU()
{
    if (canBus == nullptr)
    {
      return;
    }

    TransponderToDcuMessage msg = {};

    // TransponderToDCUCommand is a scoped enum, so accumulate the bitmask as a
    // plain uint8_t and convert once at the end.
    uint8_t command = 0;

    if (transponder->squawkCodeUpdated)
    {
        command |= static_cast<uint8_t>(TransponderToDCUCommand::TransponderToDcuCommandSetCode);
        msg.code = static_cast<uint16_t>(transponder->getSquawkCode().toInt());
        transponder->squawkCodeUpdated = false;
    }

    if (transponder->modeUpdated) {
        command |= static_cast<uint8_t>(TransponderToDCUCommand::TransponderToDcuCommandSetMode);
        msg.mode = static_cast<uint8_t>(transponder->getMode());
        transponder->modeUpdated = false;
    }

    if (transponder->identRequest) {
        command |= static_cast<uint8_t>(TransponderToDCUCommand::TransponderToDcuCommandIdent);
        transponder->identRequest = false;
    }

    if (command != 0)
    {
        msg.command = static_cast<TransponderToDCUCommand>(command);
        canBus->sendMessage(static_cast<uint16_t>(CanMessageId::transponderInput),
                            static_cast<uint8_t>(sizeof(msg)),
                            reinterpret_cast<uint8_t*>(&msg));
    }
}
#endif

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  if (canBus != nullptr)
  {
    canBus->loop();
  }
  if (transponder != nullptr)
  {
    transponder->tick();
    sendChangesToDCU();
  }
  #endif
}
