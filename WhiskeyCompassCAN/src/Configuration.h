#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::compassNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately. D10 is the AVR /SS pin and must be an output in
// master mode; making it the chip select means SPI.begin() drives it for us.
// Left unused it would be a floating input, and a low level there drops the SPI
// out of master mode.
//
// /INT must be D2 or D3: those are the ATmega328's only external-interrupt pins
// (INT0/INT1), and BaseCAN hangs its RX handler off attachInterrupt(). On any
// other pin digitalPinToInterrupt() returns NOT_AN_INTERRUPT and attachInterrupt
// silently does nothing. Nothing else needs D2 on this board, so it gets /INT.
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// ULN2003 IN1..IN4. All four are plain digital outputs, so they deliberately sit
// on the non-PWM pins and leave every usable PWM pin for the panel lights.
const uint8_t kStepperPins[4] = {4, 7, 8, 9};

// Panel lights, 1..3 LEDs depending on how the instrument was populated. This is
// a soldering decision, so it is fixed at compile time. D3 is Timer2-backed PWM,
// D5/D6 are Timer0; the other PWM pins are unavailable (D10/D11 are SPI, D9 is a
// stepper pin). All LEDs share one brightness.
const uint8_t kLightCount = 3;
const uint8_t kLightPins[kLightCount] = {3, 5, 6};

// Compass card zero. INPUT_PULLUP, active LOW, polled — no interrupt needed, the
// homing state machine reads it between steps.
const uint8_t kHallPin = A0;

// 4096 mini-steps per card revolution, assuming the card sits directly on the
// 28BYJ-48's output shaft. If a gear reduction is fitted, this is the only value
// that changes — CompassGeometry.h is ratio-agnostic.
const uint32_t kTotalSteps = 4096L;

// Whether the card turns with or against the commanded heading depends on the
// mechanical build. Flip this once observed on the bench.
const bool kCardInversed = false;

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

// Below ~6 rpm the 28BYJ-48 overheats, above ~23 rpm it skips steps. The slow
// rate is only used for the fine Hall-window search during homing.
const double kRpmLimits[rpmKeyCount] = {
    14.0, // maxRpm
    10.0  // minRpm
};

// Offset between the Hall sensor's mechanical zero and the card's painted N mark.
// The live value lives in EEPROM (see WhiskeyCompass::Config); this is only the
// factory default a freshly flashed board starts from.
const int16_t kDefaultZeroAdjustDegree = 0;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
