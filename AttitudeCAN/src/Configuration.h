#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 0

const CanNodeId kNodeId = CanNodeId::attitudeNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10 deliberately. D10 is the AVR /SS pin and must be an output in
// master mode; making it the chip select means SPI.begin() drives it for us.
// Left unused it would be a floating input, and a low level there drops the SPI
// out of master mode.
//
// /INT must be D2 or D3: those are the ATmega328's only external-interrupt pins
// (INT0/INT1), and BaseCAN hangs its RX handler off attachInterrupt(). The old
// StepperBoard firmware had both Hall sensors on interrupt pins; here they are
// polled instead (the homing state machine reads them between steps), which
// frees D2 for CAN.
const uint8_t kCanIntPin = 2;
const uint8_t kCanCSPin = 10;

// Panel light. D5 is Timer0-backed PWM (OC0B); analogWrite there does not touch
// the Timer0 prescaler, so millis()/micros() stay correct.
const uint8_t kLightPin = 5;

enum AttitudeAxis {
    roll = 0,
    pitch,
    attitudeAxisCount
};

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

// ULN2003 IN1..IN4 per axis. Plain digital outputs; A0..A3 are full GPIO on the
// Nano. A4/A5 stay free for I2C should it ever be needed.
const uint8_t kStepperPins[attitudeAxisCount][4] = {
    {4, 7, 8, 9},       // roll
    {A0, A1, A2, A3}    // pitch
};

// Hall sensors, INPUT_PULLUP, active LOW, polled.
const uint8_t kHallPins[attitudeAxisCount] = {
    3,  // roll
    6   // pitch
};

// 4096 mini-steps per turn on both axes. The nested-axis maths in
// AttitudeGeometry.h assumes both motors share this count (display = motor +
// roll in the same step units), which is why it is one constant, not two.
const uint32_t kTotalSteps = 4096L;

// Whether a motor turns with or against its commanded direction depends on the
// wiring of the coil pins. Flip on the bench if an axis runs backwards.
const bool kStepperInversed[attitudeAxisCount] = { false, false };

// The AirManager LUA sent -roll. Kept as the default so the instrument behaves
// like before; flip if the horizon banks the wrong way on the rig.
const bool kRollInversed = true;
// Same for pitch: nose-up in the sim must read nose-up on the dial. Display
// frame only - NOT the coil flag above, which both axes' counter-move relies on.
const bool kPitchInversed = false;

// Display pitch window (LUA capped at +-30) and the gearing between display
// degrees and pitch-motor degrees (GyroDrive's kAdjustmentFactor).
const double kPitchMaxDegree = 30.0;
const double kPitchScale = 0.76923077;

// 4 rpm sits below CheapStepper's "may overheat" mark; GyroDrive used it only
// for the brief fine-search homing phases and so do we.
const double kRpmLimits[attitudeAxisCount][rpmKeyCount] = {
    {14.0, 4.0},  // roll
    {14.0, 4.0}   // pitch
};

// Factory defaults for the zero offsets (GyroDrive's kRollZeroAdjustDegree /
// kPitchZeroAdjustDegree). Live values are in EEPROM, see AttitudeIndicator.
const int16_t kDefaultZeroAdjustDegree[attitudeAxisCount] = { 1, -1 };

// Homing travel budgets in degrees, copied from GyroDrive::homeRollAxis /
// homePitchAxis. A search phase that uses up its budget without the expected
// Hall change has hit a stop or a dead sensor and fails homing.
const int16_t kRollHomingLeaveDegree = 370;       // more than a turn: sensor passed from anywhere
const int16_t kRollHomingSearchDegree = 370;
const int16_t kRollHomingSlowEndDegree = 30;
const int16_t kRollHomingSlowReturnDegree = -35;
const int16_t kRollHomingSlowStartDegree = -35;
const int16_t kPitchHomingLeaveDegree = -30;
const int16_t kPitchHomingSearchDegree = 62;
const int16_t kPitchHomingBackOffDegree = -10;
const int16_t kPitchHomingSlowSearchDegree = 15;
const int16_t kPitchHomingParkDegree = -30;       // sensor edge back to level

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
