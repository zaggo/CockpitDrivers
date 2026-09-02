#include <Arduino.h>
#include "Configuration.h"
#include "VerticalSpeedIndicator.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

VerticalSpeedIndicator* verticalSpeedIndicator;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Vertical Speed Indicator initializing..."));

  verticalSpeedIndicator = new VerticalSpeedIndicator();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(verticalSpeedIndicator);
  #else
  canBus = new CAN(verticalSpeedIndicator);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Vertical Speed Indicator started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  verticalSpeedIndicator->loop();
}
