#include <Arduino.h>
#include "Configuration.h"
#include "AirspeedIndicator.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

AirspeedIndicator* airspeedIndicator;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Airspeed Indicator initializing..."));

  airspeedIndicator = new AirspeedIndicator();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(airspeedIndicator);
  #else
  canBus = new CAN(airspeedIndicator);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Airspeed Indicator started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  airspeedIndicator->loop();
}
