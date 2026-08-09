#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0
#define DEBUGLOG_ENABLE 0

const CanNodeId kNodeId = CanNodeId::rudderNodeId;

const uint8_t kCanIntPin = 2;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
const uint8_t kCanCSPin = 53;
#else
const uint8_t kCanCSPin = 10;
#endif

// Analog inputs. All three are read against calibration points stored in EEPROM,
// so the raw ranges may run in either direction (inverted potentiometer wiring
// is handled by the mapping, not by swapping the wires).
const uint8_t kRudderPin     = A0;
const uint8_t kLeftBrakePin  = A1;
const uint8_t kRightBrakePin = A2;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
