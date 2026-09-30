#ifndef HSI_H
#define HSI_H
#include <Arduino.h>
#include <Servo.h>
#include <MCP23017.h>
#include <CheapStepper.h>
#include "Configuration.h"

class HSI
{
public:
    enum FromTo {
        noNav = 0,
        from = 182,
        to = 90
    };
    enum HSIDriveResult {
        success = 0,
        homingError,
        notHomed,
        invalidId,
        homingTimeout,
        axisStateReached
        };
    enum HomingPhase {
        unknown = 0,
        leaveZero,
        searchZero,
        searchZeroEnd,
        returnToZeroEnd,
        searchZeroStart,
        moveToTrueZero,
        moveToAdjustedZero,
        homed,
        timeout
    };

public:
    HSI();
    ~HSI();

    bool isHomed = false;

    void loop();
    void stopAllAxes();
    void offAllAxes();

    HSIDriveResult homeAllAxis(); // Synchronous
    HSIDriveResult homeAxis(HSIAxis axis); // Synchronous
    HSIDriveResult moveToDegree(double obsDegree, double compassDegree, double hdgDegree, double cdiOffset, FromTo fromTo, double gsOffset); // Ansyc
    HSIDriveResult moveToDegree(HSIAxis axis, double degree);
    void moveSteps(HSIAxis axis, int16_t steps, bool synchron = false);
    HSIDriveResult moveServo(ServoId id, double degree);
    void setBrightness(uint8_t pwm);

    int16_t getObsEncoder() { return obsEncoder; }
    int16_t getHdgEncoder() { return hdgEncoder; }
    bool getObsEncoderButtonState() { return obsEncoderButtonState; }
    bool getHdgEncoderButtonState() { return hdgEncoderButtonState; }

    // Singleton instance
    static HSI* instance;
    volatile uint8_t obsEncoderState;
    volatile int16_t currentObsEncoder = 0;
    volatile uint8_t hdgEncoderState;
    volatile int16_t currentHdgEncoder = 0;
    volatile bool currentObsEncoderButtonState;
    volatile bool currentHdgEncoderButtonState;

    String errorName(HSIDriveResult result);

private:
    HSIDriveResult lookForZeroChange(HSIAxis axis, int32_t degree, bool targetZeroedState); // Synchronous
    void fetchZeroedState(HSIAxis axis);
    void readHalls();
    void moveDegree(HSIAxis axis, int16_t degree, bool synchron = false);
    int32_t calculateShortestPath(HSIAxis axis, int32_t targetPosition);
    uint32_t normalizePosition(int32_t position, uint32_t totalSteps);
    String axisName(HSIAxis axis);
    String servoName(ServoId id);
    void sendMotorData();
    HSIDriveResult nextHomingState(HSIAxis axis);
    bool checkAllHomed();
    HSIDriveResult lookForZeroChangeNonBlocking(HSIAxis axis, bool targetZeroedState);
    void resetPositionAndBacklash(HSIAxis axis);

private:
    MCP23017 *mcp;

    Servo* servos[servoCount];
    CheapStepper* axes[hsiAxisCount];
    HomingPhase homingState[hsiAxisCount];
    uint32_t zeroEndPosition[hsiAxisCount];

    uint8_t obsPattern = 0;
    uint8_t hdgPattern = 0;
    uint8_t compassPattern = 0;

    uint8_t currentPortA = 0;
    uint8_t currentPortB = 0;

    int16_t obsEncoder = 0;
    int16_t hdgEncoder = 0;
    bool obsEncoderButtonState = false;
    bool hdgEncoderButtonState = false;
    uint32_t lastObsDebounceTime = 0;
    uint32_t lastHdgDebounceTime = 0;

    bool zeroedState[hsiAxisCount] = {false, false, false};
    uint8_t hallBits = 0;

    const int32_t DEGREE_FULL_ROTATION = 360L;
};

#endif // HSI_H
