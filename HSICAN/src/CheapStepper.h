/*  
  Based on
  -----------------
  CheapStepper.h - 
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

#ifndef CHEAPSTEPPER_H
#define CHEAPSTEPPER_H

#include "Arduino.h"
#include <MCP23017.h>
#include "Backlash.h"

class CheapStepper
{

public: 
  CheapStepper(uint32_t stepsBacklash = 0, bool inversed = false);
  CheapStepper (uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, uint32_t stepsBacklash = 0, bool inversed = false);
  CheapStepper (uint8_t* patternOut, uint32_t stepsBacklash = 0,  bool inversed = false);

  uint32_t setRpm(double rpm); // sets speed (10-24 rpm, hi-low torque)
  // <6 rpm blocked in code, may overheat
  // 23-24rpm may skip

  void set4076StepMode() { totalSteps = 4076; }
  void setTotalSteps (uint32_t numSteps) { totalSteps = numSteps; } // allows custom # of steps (usually 4076)


  // non-blocking versions of move()
  // call run() in loop to keep moving
  void run(uint32_t uS = micros());
  void stop();
  void off();

  void newMove (bool clockwise, uint32_t numSteps, uint32_t uS = micros());
  void step(bool clockwise);

  int32_t getPosition() { return position; } // returns current miniStep position
  uint32_t getDelay() { return delay; } // returns current delay (microseconds)
  double getRpm() { return calcRpm(); } // returns current rpm
  int32_t getStepsLeft() { return stepsLeft; } // returns steps left in current move

  // Extension
  uint32_t getTotalSteps() { return totalSteps; }
  void resetPosition(uint32_t position = 0L);
  void setDelay(uint32_t _delay) { delay = _delay; }
  uint32_t getBacklashSteps() { return stepsBacklash; }
  void setBacklashSteps(uint32_t steps) { stepsBacklash = steps; }
  bool lastDirectionIsClockwise() { return lastDirectionClockwise; }

private:
  void stepCW () { step (true); } // move 1 step clockwise
  void stepCCW () { step (false); } // move 1 step counter-clockwise

  uint32_t calcDelay(double rpm); // calcs microsecond step delay for given rpm
  double calcRpm(uint32_t _delay); // calcs rpm for given delay in microseconds
  double calcRpm(){
    return calcRpm(delay); // calcs rpm from current delay
  }
  int32_t positionPlus(int32_t offset) {
    return (position + offset) % totalSteps;
  } // returns position + offset wrapped to totalSteps

  void incrementPosition() {
    if (consumeBacklashStep(&unusedBacklashSteps)) {
      return;
    }
    position = (position + 1) % totalSteps;
  }

  void decrementPosition() {
    if (consumeBacklashStep(&unusedBacklashSteps)) {
      return;
    }
    position = (position == 0) ? (totalSteps - 1) : (position - 1);
  }

  void seqCW();
  void seqCCW();
  void seq(uint8_t seqNum); // send specific sequence num to driver

  uint8_t pins[4]; // defaults to pins {8,9,10,11} (in1,in2,in3,in4 on the driver board)

  uint8_t* patternOut = NULL; // pointer to pattern buffer

  uint32_t stepsBacklash; // to compensate for steps lost when changing direction
  bool inversed; // for inverting stepper direction

  int32_t position = 0; // keeps track of step position
  // 0-4095 (4096 mini-steps / revolution) or maybe 4076...
  uint32_t totalSteps = 4096;

  uint32_t delay = 900; // microsecond delay between steps
  // 900 ~= 16.25 rpm
  // low speed (high torque) = 1465 ~= 1 rpm
  // high speed (low torque) = 600 ~=  24 rpm

  int8_t seqN = -1; // keeps track of sequence number

  // variables for non-blocking moves:
  uint32_t lastStepTime; // time in microseconds that last step happened
  int32_t stepsLeft = 0; // steps left to move, neg for counter-clockwise

  bool lastDirectionClockwise = true; // last direction moved
  uint32_t unusedBacklashSteps = 0; // unused backlash steps

  bool melde=false;
};

#endif
