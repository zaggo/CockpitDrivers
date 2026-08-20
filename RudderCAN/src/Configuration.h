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
const uint8_t kLeftBrakePin  = A2;
const uint8_t kRightBrakePin = A1;

// Axis deadbands, in percent of the calibrated travel they sit in. Keeping them
// proportional means the feel stays the same whatever span a calibration sweep
// measures, so re-calibrating after a mechanical change needs no re-tuning here.
// AxisMapping.h takes these as parameters and carries no tuning of its own —
// this is the only place to adjust them.

// Ignored either side of the calibrated rudder centre, as a percent of the half
// travel on that side. Absorbs the mechanical slop of the pedal linkage; without
// it the aircraft never tracks straight.
const uint8_t kRudderCenterDeadbandPercent = 2;

// Dropped at the outer end of each rudder half travel, so full deflection stays
// reachable even when the calibration sweep fell a little short of the stop.
const uint8_t kRudderEndDeadbandPercent = 3;

// Dropped at both ends of each brake travel. Smaller than the rudder's: a brake
// has no centre to hold, and full braking should arrive before the pedal bottoms
// out. Do not take this below the axis noise floor — check the `n` bench probe.
const uint8_t kBrakeEndDeadbandPercent = 2;

// Exakte ID-Matches (alle 11 Bits relevant)
const uint32_t MASK_EXACT = 0x07FF0000;

#endif // CONFIGURATION_H
