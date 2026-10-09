#include <Arduino.h>
#include "Configuration.h"
#include "TurnCoordinator.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

TurnCoordinator* turnCoordinator;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Turn Coordinator initializing..."));

  turnCoordinator = new TurnCoordinator();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(turnCoordinator);
  #else
  canBus = new CAN(turnCoordinator);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Turn Coordinator started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  turnCoordinator->loop();
}
