#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"
#include "CAN.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "DCUReceiver.h"
DCUReceiver* dcuReceiver;
#endif

CAN* canBus;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("DCU Initializing..."));
  canBus = new CAN();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(canBus);
  #else
  dcuReceiver = new DCUReceiver(canBus);
  #endif

  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("DCU started up"));
  }
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  dcuReceiver->loop();
  #endif

  canBus->loop();
}