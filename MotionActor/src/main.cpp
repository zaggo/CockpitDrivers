#include <Arduino.h>
#include "Configuration.h"
#include "DebugLog.h"
#include "MotionActor.h"
#include "CAN.h"

#if MOTION_TESTBENCH
#include "TestBench.h"
TestBench *testBench;
#endif

CAN *canBus;
MotionActor *motionActor;

void setup() {
  DEBUGLOG_INIT(kDebugBaudRate);
  delay(200);
  DEBUGLOG_PRINTLN(F("MotionActor initializing..."));

  motionActor = new MotionActor();

  canBus = new CAN(motionActor);
  #if MOTION_TESTBENCH
  testBench = new TestBench(motionActor, canBus);
  canBus->setTestBench(testBench);
  #endif
  if (canBus->begin()) {
    DEBUGLOG_PRINTLN(F("MotionActor started up"));
  }
}

void loop() {
  canBus->loop();
}
