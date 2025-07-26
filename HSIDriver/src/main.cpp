#include <Arduino.h>

#define BENCHDEBUG 0

#if BENCHDEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "AirManager.h"
AirManager* airManager;
#endif

void setup() {
  #if BENCHDEBUG
  benchDebug = new BenchDebug();
  #else
  airManager = new AirManager();
  #endif
}

void loop() {
  #if BENCHDEBUG
  benchDebug->loop();
  #else
  airManager->loop();
  #endif
}