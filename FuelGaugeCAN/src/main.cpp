#include <Arduino.h>
#include "Configuration.h"
#include "FuelGauge.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

FuelGauge* fuelGauge;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("FuleGauge initializing..."));

  fuelGauge = new FuelGauge();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(fuelGauge);
  #else
  canBus = new CAN(fuelGauge);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("FuelGauge started up"));
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