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
    if (transponder->squawkCodeUpdated)
    {
        msg.command = static_cast<TransponderToDCUCommand>(msg.command | TransponderToDcuCommandSetCode);
        msg.code = static_cast<uint16_t>(transponder->getSquawkCode().toInt());
        transponder->squawkCodeUpdated = false;
    }

    if (transponder->modeUpdated) {
        msg.command = static_cast<TransponderToDCUCommand>(msg.command | TransponderToDcuCommandSetMode);
        msg.mode = static_cast<uint8_t>(transponder->getMode());
        transponder->modeUpdated = false; 
    }

    if (transponder->identRequest) {
        msg.command = static_cast<TransponderToDCUCommand>(msg.command | TransponderToDcuCommandIdent);
        transponder->identRequest = false;
    }

    if (msg.command != 0)
    {
        canBus->sendMessage(CanMessageId::transponderInput, sizeof(msg), reinterpret_cast<uint8_t*>(&msg));
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
