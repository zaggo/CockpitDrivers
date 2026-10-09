#ifndef ATTITUDEINDICATOR_H
#define ATTITUDEINDICATOR_H
#include <Arduino.h>
#include <CheapStepper.h>
#include "Configuration.h"
#include "AttitudeGeometry.h"

class AttitudeIndicator
{
public:
    enum AttitudeResult {
        success = 0,
        notHomed,
        homingTimeout,
        axisStateReached
    };

    // Roll first, then pitch — a mechanical requirement, see beginHoming().
    enum HomingPhase {
        unknown = 0,
        rollLeaveZero,
        rollSearchZero,
        rollSearchZeroEnd,
        rollReturnToZeroEnd,
        rollSearchZeroStart,
        rollMoveToTrueZero,
        rollMoveToAdjustedZero,
        pitchLeaveZero,
        pitchSearchZero,
        pitchBackOff,
        pitchSlowSearchZero,
        pitchMoveToAdjustedZero,
        homed,
        timeout
    };

    AttitudeIndicator();

    // True once both axes found their Hall zero. Before that a setpoint means
    // nothing and would drive the display into a stop.
    bool isHomed = false;

    void loop();

    // Non-blocking: loop() drives the run forward so the CAN heartbeat keeps
    // flowing while the axes search (a run takes several seconds, well past
    // the 1500ms both ends use to declare each other dead).
    void beginHoming();
    bool isHoming() const { return homingActive; }

    // True from a failed homing run until the next beginHoming(). While set,
    // the panel light blinks instead of following the commanded brightness.
    bool hasHomingFailed() const { return homingFailed; }

    // Display-frame setpoint, degrees. Pitch is clamped to the mechanical
    // window, roll is a full circle.
    AttitudeResult moveToAttitude(double pitchDeg, double rollDeg);

    // Bench: move one axis in the display frame, the other keeps its last
    // commanded value (a roll move still counter-moves the pitch motor).
    AttitudeResult moveAxisToDegree(AttitudeAxis axis, double deg);

    void setBrightness(uint8_t brightness);

    // Records the axis' current display position as its true zero and persists
    // it (accumulating). Must be homed first.
    bool calibrateZero(AttitudeAxis axis);

    // Factory reset of both offsets. Forces a re-home.
    void wipeCalibration();

    int16_t zeroAdjustDegree(AttitudeAxis axis) const { return config.zeroAdjustDegree[axis]; }

    void stop();
    void off();

    int32_t position(AttitudeAxis axis) const { return axes[axis]->getPosition(); }
    bool hallActive(AttitudeAxis axis) const { return digitalRead(kHallPins[axis]) == LOW; }

    // What the display currently shows, in display degrees (pitch unscaled).
    double displayPitchDegree() const;
    double displayRollDegree() const;

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        int16_t zeroAdjustDegree[attitudeAxisCount];
    };

    void loadConfig();
    void saveConfig();
    void applyConfigDefaults();

    void applyLight(uint8_t brightness);
    void updateFailBlink();

    // Both motors, coordinated so they arrive together.
    void moveMotors(int32_t stepsRoll, int32_t stepsPitch);
    void moveToTargets(uint32_t rollTarget, uint32_t pitchMotorTargetPos);

    // Homing moves. Roll always drags the pitch motor the other way by the same
    // amount so the pitch display stays where it is.
    void homingMoveRollSteps(int32_t steps, RpmKeys rate);
    void homingMoveRoll(int16_t degree, RpmKeys rate);
    void homingMovePitch(int16_t degree, RpmKeys rate);
    void stopAll();
    void resetAll();

    void fetchHallStates();
    AttitudeResult lookForZeroChange(AttitudeAxis axis, bool targetState);
    AttitudeResult nextHomingState();
    void runHomingStep();
    void logHomingPhase();

    Config config;
    CheapStepper *axes[attitudeAxisCount];
    uint32_t stepDelay[attitudeAxisCount][rpmKeyCount];

    double commandedPitchDeg = 0.0;
    double commandedRollDeg = 0.0;

    HomingPhase homingState = unknown;
    bool homingActive = false;
    bool homingFailed = false;
    bool hallState[attitudeAxisCount] = { false, false };

    uint8_t commandedBrightness = 0;
    bool failBlinkOn = false;
    uint32_t failBlinkLastToggleMs = 0;
};

#endif // ATTITUDEINDICATOR_H
