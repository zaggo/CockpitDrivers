#include <Arduino.h>
#include "Configuration.h"
#include "RPMGauge.h"
#include "Odometer.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

RPMGauge* rpmGauge;
Odometer* odometer;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("RPMGauge initializing..."));

  rpmGauge = new RPMGauge();
  odometer = new Odometer();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(rpmGauge, odometer);
  #else
  canBus = new CAN(rpmGauge, odometer);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("RPMGauge started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  rpmGauge->loop();
  odometer->asyncTask();
}
