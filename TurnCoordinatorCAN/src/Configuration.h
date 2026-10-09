#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::turnCoordinatorNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately: D10 is the AVR /SS pin and must be an output in
// master mode, so making it the chip select lets SPI.begin() drive it.
// /INT must be D2 or D3 (INT0/INT1), BaseCAN hangs its RX handler off
// attachInterrupt().
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// The two needles. Index into every per-needle array below and into the
// calibration tables in EEPROM; also the order of the two int16 in CAN 0x103.
enum Needle {
    turnRate = 0,   // aircraft symbol, degrees of indicated roll
    sideSlip,       // ball, degrees of slip
    needleCount
};

// One AX1201728SG (VID6606/6608 clone) drives both X27.168: STEP/DIR per
// channel, one shared /RST. 12 steps per degree.
const uint8_t kStepPins[needleCount] = {7, 4};
const uint8_t kDirPins[needleCount] = {8, 6};
const uint8_t kRstPin = 9;

const uint16_t kMaximumDegree = 320;
const uint16_t kSteps = kMaximumDegree * 12; // 320 degrees at 1/12 degree steps

// Panel light. D3 is Timer2 PWM; analogWrite there leaves Timer0 and thus
// millis()/micros() alone.
const uint8_t kLightPin = 3;

// The degree-to-needle mapping lives entirely in EEPROM: one table of measured
// calibration points per needle (see include/TurnCoordinatorCalibration.h),
// taught on the bench with ho/ct/pt (turn rate) and ho/cs/ps (slip) and
// interpolated linearly in between. There is no compiled-in range or
// direction - both come out of the taught points. The 0 mark of either needle
// is just another taught point, not the mechanical home stop.

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
