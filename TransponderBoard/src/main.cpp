#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"

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

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  if (canBus != NULL)
  {
    canBus->loop();
  }
  if (transponder != NULL)
  {
    transponder->tick();

    if (transponder->squawkCodeUpdated || transponder->identRequest || transponder->modeUpdated)
    {
      canBus->sendTransponderState(transponder->getSquawkCode().toInt(), static_cast<uint8_t>(transponder->getMode()), transponder->identRequest ? 1 : 0);
      transponder->squawkCodeUpdated = false;
      transponder->identRequest = false;
      transponder->modeUpdated = false;
    }
  }
  #endif
}
