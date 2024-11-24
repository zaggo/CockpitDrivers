#ifndef CONFIGURATION_H
#define CONFIGURATION_H

const uint8_t kRollPin1 = 8;
const uint8_t kRollPin2 = 9;
const uint8_t kRollPin3 = 10;
const uint8_t kRollPin4 = 11;

const uint8_t kRollZeroedPin = 2;

const uint32_t kRollTotalSteps = 4096L;
const double kRollMaxRpm = 14.0;
const double kRollMinRpm = 4;
const int16_t kRollZeroAdjustDegree = 1;

const uint8_t kPitchPin1 = 4;
const uint8_t kPitchPin2 = 5;
const uint8_t kPitchPin3 = 6;
const uint8_t kPitchPin4 = 7;

const uint8_t kPitchZeroedPin = 3;

const uint32_t kPitchTotalSteps = 4096L;
const double kPitchMaxRpm = 14.0;
const double kPitchMinRpm = 4;
const int16_t kPitchZeroAdjustDegree = -1;

#endif // CONFIGURATION_H
