#include <Arduino.h>
#include "Configuration.h"
#include "Starter.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

Starter* starter;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Starter initializing..."));

  starter = new Starter();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(starter);
  #else
  canBus = new CAN(starter);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Starter started up"));
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
