#include <Arduino.h>
#include "Configuration.h"
#include "WhiskeyCompass.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

WhiskeyCompass* compass;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("WhiskeyCompass initializing..."));

  compass = new WhiskeyCompass();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(compass);
  #else
  canBus = new CAN(compass);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("WhiskeyCompass started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  compass->loop();
}
