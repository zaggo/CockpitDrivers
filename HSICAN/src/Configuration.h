#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <Arduino.h>
#include <CanNodeId.h>

#define BENCHDEBUG 1

const CanNodeId kNodeId = CanNodeId::hsiNodeId;

// CAN (MCP2515 on hardware SPI: D11 MOSI, D12 MISO, D13 SCK).
// /CS sits on D10: that is the AVR /SS pin and must be an output in master
// mode, so making it the chip select means SPI.begin() drives it for us.
//
// /INT on D2. InstrumentCAN gates its fast path on digitalRead(intPin) and
// drains while the line is low, so the wire is load-bearing even though the
// ISR is not - the DCU proves that, running kCanIntPin = 48 on a Mega where
// attachInterrupt silently does nothing. D2 keeps the interrupt working too.
const uint8_t kCanCSPin = 10;
const uint8_t kCanIntPin = 2;

// Panel light. D6 is Timer0-backed PWM (OC0A) and is the only PWM pin left:
// the Servo library owns Timer1 and takes D9/D10 with it, D11 is MOSI, and
// D3/D5 are encoder inputs. analogWrite on D6 sets COM0A1 only and does not
// touch the Timer0 prescaler, so millis()/micros() are unaffected.
const uint8_t kLightPin = 6;

// Exact ID matches (all 11 bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

const uint8_t kMCP23017Address = 0x20;

// The OBS knob sets the course pointer.
const uint8_t kObsEncoderAPin = 3;
const uint8_t kObsEncoderBPin = 4;
const uint8_t kObsEncoderPushButtonPin = 5;
const uint8_t kObsEncoderInterruptMask = (1 << PCINT19) | (1 << PCINT20) | (1 << PCINT21); // for PCMSK2!

// The HDG knob sets the heading bug. It was called the "compass" encoder, which
// it never was - it does not touch the card.
const uint8_t kHdgEncoderAPin = A1;
const uint8_t kHdgEncoderBPin = A2;
const uint8_t kHdgEncoderPushButtonPin = A3;
const uint8_t kHdgEncoderInterruptMask = (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11); // for PCMSK1!

enum ServoId {
    cdiServo = 0,
    fromToServo,
    gs1Servo,
    gs2Servo,
    servoCount
};

enum HSIAxis {
    obs = 0,
    compass,
    hdg,
    hsiAxisCount
};

enum RpmKeys {
    maxRpm = 0,
    minRpm,
    rpmKeyCount
};

// Three servos moved off the SPI pins; D6 went to the panel light, so the
// fourth landed on A0 - a full digital pin, freed by the halls moving to the
// port extender.
const uint8_t kServoPins[servoCount] = {
    9,   // cdiServo   (unchanged)
    8,   // fromToServo (was D11 / MOSI)
    7,   // gs1Servo    (was D10 / SS)
    A0   // gs2Servo    (was D12 / MISO)
};

// All three hall sensors moved to MCP23017 port B. A6/A7 have no internal
// pull-up and the PCB has no external ones, the extender does; and because
// all three land on one port, a single readPort(B) returns them together.
// GPB0..3 remain the compass stepper's coils.
const uint8_t kHallBits[hsiAxisCount] = {
    0x10, // obs     -> GPB4 (was D2)
    0x20, // compass -> GPB5 (was A0)
    0x40  // hdg     -> GPB6 (was D8)
};
const uint8_t kHallMask = 0x70;

const uint32_t kTotalSteps[hsiAxisCount] = {
    4096L, 4096L, 4096L
};

const int16_t kZeroAdjustDegree[hsiAxisCount] = {
    -90,85,11
    // 92, // OBS
    // 128, // Compass
    // -115 // HDG
};

const double kServoAdjustDegree[servoCount] = {
    80., // cdiServo
    0., // From-To Servo
    90.-19., // gs1Servo
    90.-1. // gs2Servo
};

const double kServoAdjustRatio[servoCount] = {
    2., // cdiServo
    1., // From-To Servo
    2., // gs1Servo
    -10./7.  // gs2Servo
};

const double kServoMinimumDegree[servoCount] = {
    -40., // cdiServo
    -999., // From-To Servo
    -40., // gs1Servo
    -40.  // gs2Servo
};

const double kServoMaximumDegree[servoCount] = {
    40., // cdiServo
    999., // From-To Servo
    40., // gs1Servo
    40.  // gs2Servo
};


const double kRpmLimits[hsiAxisCount][rpmKeyCount] = {
    // OBS
    {12.0, 10.0},
    // Compass
    {12.0, 10.0},
    // Bug
    {12.0, 10.0}
};

#endif // CONFIGURATION_H
