#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"
#include "MotionActor.h"

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "CAN.h"
CAN *canBus;
#endif

MotionActor *motionActor;

void setup() {
  DEBUGLOG_INIT(115200);
  delay(200);
  DEBUGLOG_PRINTLN(F("MotionActor initializing..."));

  motionActor = new MotionActor();

  #if BENCHDEBUG
  benchDebug = new BenchDebug(motionActor);
  #else
  canBus = new CAN(motionActor);
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("MotionActor started up"));
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