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
        // Taught (dots, percent) tables, index 0 = gs1Servo, 1 = gs2Servo.
        GsPoint  gsPoints[2][kGsPointCount];
        uint8_t  gsPointCount[2];
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
    void beginHoming(HSIAxis axis);
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
    void setDegPerDot(int16_t degPerDot100);

    // GS servo table (gs1Servo / gs2Servo only). setGsServoPercent drives the
    // servo to an absolute deflection around its 50 % centre, bypassing the
    // sa clamp/ratio; addGsPoint stores (dots, percent) for interpolation.
    void setGsServoPercent(ServoId id, int8_t percent);
    bool addGsPoint(ServoId id, int16_t dots1000, int8_t percent);
    void clearGsPoints(ServoId id);

    // Bench only: blocking, and there is no CAN loop running in a bench build.
    uint32_t measureBacklashSteps(HSIAxis axis);

    // Bench only: hall bits as last read in loop() (set = at zero), and the
    // MCP's port B config read back from the chip.
    uint8_t getHallBits() const { return hallBits; }
    uint8_t readHallRegister(MCP23017Register reg) { return mcp->readRegister(reg); }

    int16_t getObsEncoder() { return obsEncoder; }
    int16_t getHdgEncoder() { return hdgEncoder; }
    bool getObsEncoderButtonState() { return obsEncoderButtonState; }
    bool getHdgEncoderButtonState() { return hdgEncoderButtonState; }

    // Encoder movement since the last call, as signed clicks. Returns false
    // when nothing moved. The board keeps no absolute position: the pointers
    // follow whatever the sim reports back over 0x104.
    bool takeObsDelta(int16_t* delta);
    bool takeHdgDelta(int16_t* delta);

    // True once per completed short press of the HDG knob.
    bool takeHdgShortPress();

    // The build-time RAM figure counts statics only; this is the gap between
    // the top of the heap and the current stack pointer at the time of the
    // call - what actually ran out on RPMGaugeCAN.
    static int freeRam();

    // Singleton instance
    static HSI* instance;
    volatile uint8_t obsEncoderState;
    volatile int16_t currentObsEncoder = 0;
    volatile uint8_t hdgEncoderState;
    volatile int16_t currentHdgEncoder = 0;
    volatile bool currentObsEncoderButtonState;
    volatile bool currentHdgEncoderButtonState;

private:
    void fetchZeroedState(HSIAxis axis);
    void readHalls();
    void moveDegree(HSIAxis axis, int16_t degree, bool synchron = false);
    int32_t calculateShortestPath(HSIAxis axis, int32_t targetPosition);
    uint32_t normalizePosition(int32_t position, uint32_t totalSteps);
    void sendMotorData();
    HSIDriveResult nextHomingState(HSIAxis axis);
    void runHoming();
    bool checkAllHomed();
    HSIDriveResult lookForZeroChangeNonBlocking(HSIAxis axis, bool targetZeroedState);
    void loadConfig();
    void moveGs(int16_t dots1000);

private:
    HsiConfig config;
    MCP23017 *mcp;

    Servo* servos[servoCount];
    CheapStepper* axes[hsiAxisCount];
    // Idle until beginHoming() says otherwise. loop() evaluates isHoming() from
    // the very first iteration, and this object is heap-allocated, so an
    // uninitialised phase here is read before anything writes it. timeout, not
    // homed: with homed, homing a single axis after boot reported the whole
    // instrument homed and let hd/ob/bg drive the two unreferenced axes.
    HomingPhase homingState[hsiAxisCount] = {timeout, timeout, timeout};
    uint32_t zeroEndPosition[hsiAxisCount] = {0, 0, 0};

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

    int16_t lastSentObsEncoder = 0;
    int16_t lastSentHdgEncoder = 0;
    uint32_t hdgPressStartedAt = 0;
    bool hdgPressActive = false;
    bool hdgShortPressPending = false;

    bool zeroedState[hsiAxisCount] = {false, false, false};
    uint8_t hallBits = 0;
    bool homingFailure = false;
    bool gsValid = false;

    const int32_t DEGREE_FULL_ROTATION = 360L;
};

#endif // HSI_H
