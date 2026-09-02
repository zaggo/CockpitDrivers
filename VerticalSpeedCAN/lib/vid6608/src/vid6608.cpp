/**
 * VID6608 gauge motor driver library
 * by Petr Golovachev <petro@petro.ws>, 2025
 *
 * https://github.com/petrows/arduino-vid6608
 *
 * Licensed under the GPL, see LICENSE.txt for details.
 *
 * LOCAL FORK (RPMGaugeCAN) - see vid6608.h header for rationale.
 */

#include "vid6608.h"

// This table defines the acceleration curve.
// 1 value: distance from begin / target in steps
// 2 value: speed delay in microseconds
static vid6608::AccelTable defaultAccelTable[] = {
  {30, 3000},
  {65, 2920},
  {100, 2780},
  {135, 2600},
  {170, 2380},
  {205, 2140},
  {240, 1890},
  {275, 1650},
  {310, 1420},
  {345, 1210},
  {380, 1020},
  {415, 860},
  {450, 730},
  {485, 620},
  {520, 530},
  {555, 460},
  {590, 410},
  {625, 370},
  {660, 340},
  {695, 320},
  {730, 310},
  {765, 305},
  {800, 300},
};

vid6608::vid6608(int stepPin, int dirPin, uint16_t maxSteps /*= VID6608_MAX_STEPS*/) {
  // Zero values:
  this->maxSteps = maxSteps;
  this->stepPin = stepPin;
  this->dirPin = dirPin;
  this->moveState = STATE_IDLE;
  this->moveDirection = MOVE_NONE;
  this->dirPinState = MOVE_NONE; // invalid state to force update on first step
  this->currentPosition = 0;
  this->targetPosition = 0;
  this->targetPositionNext = 0;
  this->lastStepMicros = 0;
  this->setAccelTable(defaultAccelTable);
  // Setup pins
  pinMode(this->stepPin, OUTPUT);
  pinMode(this->dirPin, OUTPUT);
  digitalWrite(this->stepPin, LOW);
  digitalWrite(this->dirPin, LOW);
}

void vid6608::zero(uint16_t initialPos /*= VID6608_DEFAULT_MAX_STEPS/2*/, uint16_t delay /*= VID6608_DEFAULT_ZERO_SPEED*/) {
  // We have to optimize the zeroing process to avoid bouncing on end-stops
  // Drive makes 1/2 move forward and 1/2 backward to reduce bouncing
  // This will reduce bouncing, if the last position was not a zero position
  if (initialPos >= this->maxSteps) {
    initialPos = this->maxSteps - 1;
  }
  uint16_t stepsForward = this->maxSteps - initialPos;
  // Move forward to defined steps count
  for (uint16_t x = 0; x < stepsForward; x++) {
    step(MOVE_FORWARD, delay);
  }
  // Move full round back to 0
  for (uint16_t x = 0; x < this->maxSteps; x++) {
    step(MOVE_BACKWARD, delay);
  }
  // Move back for more steps with low speed for precise zero set, if allowed
  if (VID6608_ZERO_PULLUP_STEPS) {
    for (uint16_t x = 0; x < VID6608_ZERO_PULLUP_STEPS; x++) {
      step(MOVE_BACKWARD, delay * 10);
    }
  }
  // Reset values
  this->currentPosition = 0;
  this->targetPosition = 0;
  this->targetPositionNext = 0;
  this->moveState = STATE_IDLE;
  this->moveDirection = MOVE_NONE;
}

void vid6608::moveTo(uint16_t position) {
  // Test for duplicates
  if (position == this->targetPositionNext) {
    return;
  }
  // Sanity check
  if (position > this->maxSteps - 1) {
    position = this->maxSteps - 1;
  }
  // Calculate new values to move and schedule it for next async call
  this->targetPositionNext = position;
}

bool vid6608::isMoving() {
  return this->moveState == STATE_MOVING;
}

uint16_t vid6608::getPosition() {
  return this->currentPosition;
}

void vid6608::loop() {
  // Check if we have a new target position scheduled
  if (this->targetPosition != this->targetPositionNext) {
    MoveDirection newDir = this->targetPositionNext > this->currentPosition
                               ? MOVE_FORWARD
                               : (this->targetPositionNext < this->currentPosition ? MOVE_BACKWARD : MOVE_NONE);

    if (this->moveState == STATE_IDLE) {
      // Start a fresh move from rest.
      this->targetPosition = this->targetPositionNext;
      if (newDir == MOVE_NONE) {
        // Already at the target - nothing to do, just clear the schedule.
      } else {
        this->moveState = STATE_MOVING;
        this->moveDirection = newDir;
        this->moveDone = 0;
        // Calculate point values, we have to save 1/2 of way to compare it,
        // it is required to decide what distance we have (from begin or to end)
        if (this->moveDirection == MOVE_FORWARD) {
          this->moveLeft = this->targetPosition - this->currentPosition;
        } else {
          this->moveLeft = this->currentPosition - this->targetPosition;
        }
        // LOCAL FORK: start the step clock fresh so the first step of this move
        // fires promptly rather than inheriting a stale (possibly huge) delta.
        this->lastStepMicros = micros();
      }
    } else if (newDir == this->moveDirection) {
      // LOCAL FORK: on-the-fly retarget. The setpoint moved further in the
      // SAME direction while we're already rolling (the common case when a
      // 50Hz feed keeps nudging the target out). Extend the target and
      // recompute distance-to-end WITHOUT resetting moveDone, so we stay at
      // cruise speed instead of decelerating to a stop at every waypoint.
      // moveDone stays large -> accelDistance is governed by moveLeft, which
      // only shrinks (triggering decel) once the setpoint finally settles.
      this->targetPosition = this->targetPositionNext;
      this->moveLeft = (this->moveDirection == MOVE_FORWARD)
                           ? this->targetPosition - this->currentPosition
                           : this->currentPosition - this->targetPosition;
    }
    // Opposite-direction request while moving: leave it scheduled. The current
    // move finishes (decel to its target), then the STATE_IDLE branch above
    // picks up the reversal next time around.
  }
  // If we are moving -> update to next position
  if (this->moveState == STATE_MOVING) {
    // Check what we have - close to end or begin?
    uint16_t accelDistance = 0;
    if (this->moveDone < this->moveLeft) {
      // We are in the first half
      accelDistance = this->moveDone;
    } else {
      // We are in the second half
      accelDistance = this->moveLeft;
    }
    // Get the actual speed, depending on distance from/to
    uint16_t accelDelay = getDelay(accelDistance);

    // LOCAL FORK: non-blocking gate. Only step once the accel delay has
    // elapsed since the previous pulse. Everything else the caller does
    // between loop() calls (OLED I2C, CAN) overlaps this wait instead of
    // stretching the step period.
    uint32_t now = micros();
    if ((uint32_t)(now - this->lastStepMicros) < accelDelay) {
      return;
    }
    // Phase-accumulate so the long-run cadence stays exactly accelDelay even
    // if we noticed a bit late. If we fell more than one full interval behind
    // (e.g. a blocking section held us off), resync to now to avoid a burst
    // of catch-up steps.
    this->lastStepMicros += accelDelay;
    if ((uint32_t)(now - this->lastStepMicros) > accelDelay) {
      this->lastStepMicros = now;
    }

    // Actual move
    stepPulse(this->moveDirection);
    this->moveDone++;
    this->moveLeft--;
    this->currentPosition += (this->moveDirection == MOVE_FORWARD) ? 1 : -1;
    // Check the end of movement
    if (this->currentPosition == this->targetPosition) {
      this->moveState = STATE_IDLE;
    }
  }
}

void vid6608::step(vid6608::MoveDirection direction, uint16_t delayUs) {
  if (direction != this->dirPinState) {
    this->dirPinState = direction;
    digitalWrite(this->dirPin, direction == MOVE_FORWARD ? LOW : HIGH);
    // Setup time must be > 100ns, we 1us to be safe
    delay(1);
  }
  digitalWrite(this->stepPin, HIGH);
  delayMicroseconds(delayUs);
  // VID6608 reacts on raising front, so we can lower the pin immediately with lower delay
  // to improve max speed. Lower time must be > 100ns, we set 1us to be safe.
  digitalWrite(this->stepPin, LOW);
  delayMicroseconds(1);
  // We should keep resources reserved by others
  yield();
}

void vid6608::stepPulse(vid6608::MoveDirection direction) {
  // LOCAL FORK: short, non-blocking step pulse. The inter-step interval is
  // enforced by loop()'s micros() gate, so this only has to emit a valid
  // rising edge - no busy-wait for the whole accel delay.
  if (direction != this->dirPinState) {
    this->dirPinState = direction;
    digitalWrite(this->dirPin, direction == MOVE_FORWARD ? LOW : HIGH);
    // Direction setup time only needs > 100ns. The original used delay(1),
    // which is 1 MILLISECOND (not 1us as its comment claimed) - a real stall
    // on every direction change. A couple of microseconds is plenty.
    delayMicroseconds(2);
  }
  // VID6608 reacts on the rising edge; pulse width just needs to be > 100ns.
  digitalWrite(this->stepPin, HIGH);
  delayMicroseconds(2);
  digitalWrite(this->stepPin, LOW);
}

uint16_t vid6608::getDelay(uint16_t distance) {
  if (distance >= this->accelMaxDistance) {
    return this->accelMaxDelay;
  }
  for (uint16_t x = 0; x < this->accelTableSize; x++) {
    if (this->accelTable[x].distance > distance) {
      return this->accelTable[x].delay;
    }
  }
  // We should never be here, but return to be safe
  return this->accelMaxDelay;
}
