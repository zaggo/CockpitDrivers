#include <Arduino.h>
#include "Configuration.h"
#include "AttitudeIndicator.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

AttitudeIndicator* attitude;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("AttitudeIndicator initializing..."));

  attitude = new AttitudeIndicator();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(attitude);
  #else
  canBus = new CAN(attitude);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("AttitudeIndicator started up"));
  } else {
    // InstrumentCAN::loop() early-returns forever once begin() failed, so the
    // board goes quiet rather than obviously dead. Say so on the console.
    DEBUGLOG_PRINTLN(F("AttitudeIndicator CAN startup FAILED - board is inert"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  attitude->loop();
}
