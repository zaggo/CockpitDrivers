#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0                                           
#define DEBUGLOG_ENABLE 1

const CanNodeId kNodeId = CanNodeId::handbrakeNodeId;

const uint8_t kCanIntPin = 2;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
const uint8_t kCanCSPin = 53;
#else   
const uint8_t kCanCSPin = 10;
#endif

const uint8_t kHandbrakePin = A0;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H