#include <Arduino.h>
#include "Configuration.h"
#include "HSI.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

HSI* hsi;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("HSI initializing..."));

  hsi = new HSI();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(hsi);
  #else
  canBus = new CAN(hsi);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("HSI started up"));
  }
  #endif

  DEBUGLOG_PRINT(F("Free RAM: "));
  DEBUGLOG_PRINTLN(HSI::freeRam());
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  hsi->loop();
}
