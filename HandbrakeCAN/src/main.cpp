#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

Handbrake* handbrake;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Handbrake initializing..."));

  handbrake = new Handbrake();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(handbrake);
  #else
  canBus = new CAN(handbrake);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Handbrake started up"));
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