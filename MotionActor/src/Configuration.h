#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <MotionNodeId.h>

#define BENCHDEBUG 0                                           

const MotionNodeId kNodeId = MotionNodeId::actorPair2;
const byte kActorAddress = 129; // Default Packet Serial address for Kangaroo is 128. Adjust if needed.
const uint32_t kKangarooBaudRate = 19200; // Default baud rate for Kangaroo. Adjust if needed.

const uint8_t kCanIntPin = 2;
#if defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
const uint8_t kCanCSPin = 53;
#else   
const uint8_t kCanCSPin = 10;
#endif

const uint8_t kRedLEDPin = 4; // LED on if CAN error/not initialized
const uint8_t kGreenLEDPin = 3; // LED 

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H