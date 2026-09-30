#ifndef HSI_H
#define HSI_H
#include <Arduino.h>
#include <Servo.h>
#include <MCP23017.h>
#include <CheapStepper.h>
#include "Configuration.h"
#include "HSICalibration.h"

class HSI
{
public:
    struct HsiConfig
    {
        uint16_t magic;
        int16_t  zeroAdjustDegree[hsiAxisCount];
        uint16_t stepsBacklash[hsiAxisCount];
        uint16_t moveDeadband[hsiAxisCount];
        uint16_t reversalDeadband[hsiAxisCount];
        int16_t  servoAdjustDegree100[servoCount];
        int16_t  servoAdjustRatio1000[servoCount];
        int16_t  cdiDegPerDot100;
        int16_t  gsDegPerDot100;
    };

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
        takeUpBacklash = 0,
        unknown,
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

    void beginHoming();
    bool isHoming() const;
    bool homingFailed() const;
    void applyHeading(uint16_t heading100, uint16_t obs100, uint16_t hdgBug100);
    void applyFlags(uint8_t flags);
    void applyDeviation(int16_t cdiDots1000, int16_t gsDots1000);
    void park();
    HSIDriveResult moveToDegree(HSIAxis axis, double degree);
    void moveSteps(HSIAxis axis, int16_t steps, bool synchron = false);
    HSIDriveResult moveServo(ServoId id, double degree);
    void setBrightness(uint8_t pwm);

    const HsiConfig& calibration() const { return config; }
    void saveConfig();
    void wipeCalibration();
    bool calibrateZero(HSIAxis axis);
    void setStepsBacklash(HSIAxis axis, uint16_t steps);
    void setServoAdjust(ServoId id, int16_t degree100, int16_t ratio1000);
    void setDegPerDot(bool isGs, int16_t degPerDot100);

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
    void fetchZeroedState(HSIAxis axis);
    void readHalls();
    void moveDegree(HSIAxis axis, int16_t degree, bool synchron = false);
    int32_t calculateShortestPath(HSIAxis axis, int32_t targetPosition);
    uint32_t normalizePosition(int32_t position, uint32_t totalSteps);
    String axisName(HSIAxis axis);
    String servoName(ServoId id);
    void sendMotorData();
    HSIDriveResult nextHomingState(HSIAxis axis);
    void runHoming();
    bool checkAllHomed();
    HSIDriveResult lookForZeroChangeNonBlocking(HSIAxis axis, bool targetZeroedState);
    void loadConfig();

private:
    HsiConfig config;
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
    bool homingFailure = false;
    bool gsValid = false;

    const int32_t DEGREE_FULL_ROTATION = 360L;
};

#endif // HSI_H
