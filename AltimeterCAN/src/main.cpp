#include <Arduino.h>
#include "Configuration.h"
#include "Altimeter.h"
#include "DebugLog.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN* canBus;
#endif

Altimeter* altimeter;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Altimeter initializing..."));

  altimeter = new Altimeter();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(altimeter);
  #else
  canBus = new CAN(altimeter);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Altimeter started up"));
  }
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  canBus->loop();
  #endif
  altimeter->loop();
}
