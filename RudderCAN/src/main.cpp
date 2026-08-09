#include <Arduino.h>
#include "Configuration.h"
#include "Rudder.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

Rudder* rudder;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Rudder initializing..."));

  rudder = new Rudder();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(rudder);
  #else
  canBus = new CAN(rudder);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Rudder started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
}
