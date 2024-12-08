#include <Arduino.h>

// Only if we're running on a Duemillanove...
#if defined(__AVR_ATmega328P__)
#include "BenchDebug.h"
BenchDebug* benchDebug;
#else
#include "AirManager.h"
AirManager* airManager;
#endif

void setup() {
  #if defined(__AVR_ATmega328P__)
  benchDebug = new BenchDebug(kTachometer);
  #else
  airManager = new AirManager();
  #endif
  delay(100);
}

void loop() {
  #if defined(__AVR_ATmega328P__)
  benchDebug->loop();
  #else
  airManager->loop();
  #endif
}
