/*  
  Based on
  -----------------
  CheapStepper.cpp - 
  v0.2
  Library for the 28BYJ-48 stepper motor, using ULN2003 driver board
  https://arduino-info.wikispaces.com/SmallSteppers

  Library written by Tyler Henry, 6/2016

  uses 8-step sequence: A-AB-B-BC-C-CD-D-DA

  motor has gear ratio of either:
    64:1 (per manufacturer specs)
    or 
    63.68395:1 measured
      (see: http://forum.arduino.cc/index.php?topic=71964.15)
  * 64 steps per internal motor rev
  = 

  4096 total mini-steps / revolution
  or ~4076 (4075.7728) depending on exact gear ratio

  assumes 5v power source for rpm calc
*/


#include "Arduino.h"
#include "CheapStepper.h"

CheapStepper::CheapStepper() : pins{ 8, 9, 10, 11 } {
  for (int pin=0; pin<4; pin++){
    pinMode(pins[pin], OUTPUT);
  }
}

CheapStepper::CheapStepper(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) : pins{ in1, in2, in3, in4 } {
  for (uint8_t pin=0; pin<4; pin++){
    pinMode(pins[pin], OUTPUT);
  }
}

uint32_t CheapStepper::setRpm(double rpm) {
  delay = calcDelay(rpm);
  return delay;
}

void CheapStepper::move(bool clockwise, uint32_t numSteps) {
  for (uint32_t n=0; n<numSteps; n++){
    step(clockwise);
  }
}

void CheapStepper::moveTo(bool clockwise, uint32_t toStep) {
  // keep to 0-(totalSteps-1) range
  toStep %= totalSteps;
  while (position != toStep){
    step(clockwise);
  }
}

void CheapStepper::moveDegrees(bool clockwise, uint16_t deg) {
  int nSteps = static_cast<uint32_t>(deg) * totalSteps / 360L;
  move(clockwise, nSteps);
}

void CheapStepper::moveToDegree(bool clockwise, uint16_t deg) {
  // keep to 0-359 range
  deg %= 360;
  uint32_t toStep = static_cast<uint32_t>(deg) * totalSteps / 360L;
  moveTo (clockwise, toStep);
}


// NON-BLOCKING MOVES

void CheapStepper::newMove(bool clockwise, uint32_t numSteps, uint32_t currentMicros) {
  // numSteps sign ignored
  // stepsLeft signed positive if clockwise, neg if ccw

  if (clockwise) stepsLeft = static_cast<int32_t>(numSteps);
  else stepsLeft = - static_cast<int32_t>(numSteps);

  lastStepTime = currentMicros;
}

void CheapStepper::newMoveTo (bool clockwise, uint32_t toStep, uint32_t currentMicros) {
  // keep toStep in 0-(totalSteps-1) range
  toStep %= totalSteps;

  if (clockwise) stepsLeft = static_cast<int32_t>(abs(toStep - position));
  // clockwise: simple diff, always pos
  else stepsLeft = -(static_cast<int32_t>(totalSteps) - static_cast<int32_t>(abs(toStep - position)));
  // counter-clockwise: totalSteps - diff, made neg

  lastStepTime = currentMicros;
}

void CheapStepper::newMoveDegrees(bool clockwise, uint16_t deg) {
  uint32_t nSteps = static_cast<uint32_t>(deg) * totalSteps / 360L;
  newMove (clockwise, nSteps);
}

void CheapStepper::newMoveToDegree(bool clockwise, uint16_t deg) {
  // keep to 0-359 range
  deg %= 360;

  uint32_t toStep = static_cast<int32_t>(deg) * totalSteps / 360L;
  newMoveTo (clockwise, toStep);
}


void CheapStepper::run(uint32_t currentMicros) {
  if (lastStepTime + delay <= currentMicros) { // if time for step
    if (stepsLeft > 0) { // clockwise
      stepCW();
      stepsLeft--;
    } else if (stepsLeft < 0){ // counter-clockwise
      stepCCW();
      stepsLeft++;
    } else {
      // off();
    }
    lastStepTime = currentMicros;
   }
}

void CheapStepper::stop() {
  stepsLeft = 0;
}


void CheapStepper::step(bool clockwise) {
  if (clockwise) seqCW();
  else seqCCW();
}

void CheapStepper::off() {
  for (uint8_t p=0; p<4; p++)
    digitalWrite(pins[p], 0);
}

/////////////
// PRIVATE //
/////////////

uint32_t CheapStepper::calcDelay(double rpm) {
  uint32_t d = 60000000L / static_cast<uint32_t>(static_cast<double>(totalSteps) * rpm);
  // nominal range: 600-1465 microseconds (24-1 rpm)
  return d;
}

double CheapStepper::calcRpm (uint32_t _delay) {
  double rpm = 60000000.0 / static_cast<double>(_delay) / static_cast<double>(totalSteps);
  return rpm;
}

void CheapStepper::seqCW() {
  seqN++;
  if (seqN > 7) seqN = 0; // roll over to A seq
  seq(seqN);

  position++; // track miniSteps
  if (position >= totalSteps){
    position = 0; // keep stepN within 0-(totalSteps-1)
  }
}

void CheapStepper::seqCCW() {
   seqN--;
  if (seqN < 0) seqN = 7; // roll over to DA seq
  seq(seqN);

  // track miniSteps
  if (position == 0) {
    position = totalSteps-1; // keep stepN within 0-(totalSteps-1)
  } else {
    position--;
  }
}

void CheapStepper::seq(uint8_t seqNum) {
  int pattern[4];
  // A,B,C,D HIGH/LOW pattern to write to driver board
  
  switch(seqNum){
    case 0:
    {
      pattern[0] = 1;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
    }
    case 1:
    {
      pattern[0] = 1;
      pattern[1] = 1;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
    }
    case 2:
    {
      pattern[0] = 0;
      pattern[1] = 1;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
    }
    case 3:
    {
      pattern[0] = 0;
      pattern[1] = 1;
      pattern[2] = 1;
      pattern[3] = 0;
      break;
    } 
    case 4:
    {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 1;
      pattern[3] = 0;
      break;
    }
    case 5:
    {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 1;
      pattern[3] = 1;
      break;
    }
    case 6:
    {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 1;
      break;
    }
    case 7:
    {
      pattern[0] = 1;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 1;
      break;
    }
    default:
    {
      pattern[0] = 0;
      pattern[1] = 0;
      pattern[2] = 0;
      pattern[3] = 0;
      break;
    }
  }

  // write pattern to pins
  for (uint8_t p=0; p<4; p++){
    digitalWrite(pins[p], pattern[p]);
  }
}
