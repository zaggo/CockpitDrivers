#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"
#include "CAN.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "MotionGateway.h"
MotionGateway* motionGateway;
#endif

CAN* canBus;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("Motion Gateway Initializing..."));
  canBus = new CAN();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(canBus);
  #else
  motionGateway = new MotionGateway(canBus);
  #endif

  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("Motion Gateway started up"));
  }
  DEBUGLOG_PRINTLN(F("Waiting for data..."));
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  motionGateway->loop();
  #endif

  canBus->loop();
}