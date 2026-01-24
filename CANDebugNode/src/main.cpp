#include <Arduino.h>
#include <Wire.h>
#include "Configuration.h"
#include "DebugLog.h"
#include "CAN.h"
#include "LCD.h"
CAN *canBus;
LCD *lcd;

void setup()
{
   DEBUGLOG_INIT(115200);
   delay(200);
   DEBUGLOG_PRINTLN(F("CanDebug initializing..."));

   lcd = new LCD();
   if (lcd->begin())
   {
      lcd->printFirstLine(F("CAN Debug Node"));
      lcd->printSecondLine(F("Starting..."));
   } else {
      DEBUGLOG_PRINTLN(F("LCD init failed"));
   }
   canBus = new CAN(lcd);
   if (canBus->begin())
   {
      DEBUGLOG_PRINTLN(F("FuelGauge started up"));
   }
   else
   {
      DEBUGLOG_PRINTLN(F("FuelGauge failed to start"));
   }
}

void loop() {
  canBus->loop();
}