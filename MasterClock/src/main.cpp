#include <Arduino.h>

#define BENCH_DEBUG 1

#if BENCH_DEBUG
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "AirManager.h"
AirManager* airManager;
#endif

void setup() {
  #if BENCH_DEBUG
  benchDebug = new BenchDebug(kTachometer);
  #else
  airManager = new AirManager();
  #endif
}

void loop() {
  #if BENCH_DEBUG
  benchDebug->loop();
  #else
  airManager->loop();
  #endif
}
