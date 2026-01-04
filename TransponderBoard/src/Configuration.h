#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <avr/pgmspace.h>

#define BENCHDEBUG 0

const uint32_t kHeartbeatInterval = 1000L; // 1 second

const uint8_t kTransponderClkPin = 4;
const uint8_t kTransponderDioPin = 3;

const uint8_t kSDA = A4;
const uint8_t kSCL = A5;

const uint8_t kMCP23017Address = 0x20; // Default I2C address
const uint8_t kMCP23017InterruptPin = 2; // Pin connected to MCP23017 INT output

const uint8_t kLEDDigits = 6;

// const uint32_t kPwrButtonLongPressDuration = 2000L; // 2 seconds
// const uint32_t kIdentDuration = 3000L;               // 3 seconds
// const uint32_t kSquawkCommitDelay = 200L;          // 0.2 seconds
// const uint32_t kSquawkEntryTimeout = 3000L;      // 3 seconds
// const uint32_t kSquawkEntryBlinkInterval = 500L; // 0.5 seconds

#endif // CONFIGURATION_H
