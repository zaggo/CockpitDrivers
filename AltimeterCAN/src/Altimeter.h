#ifndef ALTIMETER_H
#define ALTIMETER_H
#include <Arduino.h>
#include <Servo.h>
#include <MCP23017.h>
#include "CheapStepper.h"
#include "Configuration.h"
#include "AltimeterCalibration.h"

class Altimeter
{
public:
    enum AltimeterDriveResult {
        success = 0,
        homingError,
        notHomed,
        invalidId,
        coupledAxisError,
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
    Altimeter();
    ~Altimeter();

    // Returns true, if all axes homingState are `homed`
    bool isHomed = false;

    // Raw ADC reading of the barometer pot, and that reading run through the
    // stored calibration. inHg * 100, matching CAN 0x340.
    uint16_t baroRaw() const;
    uint16_t baroInHg100Now() const;

    void loop();
    void stopAllAxes();
    void offAllAxes();

    AltimeterDriveResult homeAllAxis(); // Synchronous
    AltimeterDriveResult homeAxis(AltimeterAxis axis); // Synchronous

    // Starts a homing run that is driven forward by loop(). Unlike homeAllAxis()
    // this returns immediately, so the CAN heartbeat keeps flowing in both
    // directions while the axes search — a blocking run takes several seconds,
    // well past the 1500ms both ends use to declare each other dead.
    void beginHoming();
    bool isHoming() const { return homingActive; }

    // Records the current position of `axis` as its true zero and persists it.
    // Must be homed first; the axis is re-zeroed on success.
    bool calibrateZero(AltimeterAxis axis);

    // Factory reset: needle offsets and baro endpoints back to the compiled-in
    // defaults.
    void wipeCalibration();

    int16_t zeroAdjustDegree(AltimeterAxis axis) const { return config.zeroAdjustDegree[axis]; }
    const BaroCalibration &baroCalibration() const { return config.baro; }

    // Stores the pot's current raw reading against `inHg100` as the low or high
    // endpoint.
    void setBaroCalibrationPoint(bool isHigh, uint16_t inHg100);

    AltimeterDriveResult moveToDegree(double hundredDegree, double thousandDegree, double tenThousandDegree, double flagDegree); // Ansyc
    AltimeterDriveResult moveToDegree(AltimeterAxis axis, double degree);
    AltimeterDriveResult moveServo(ServoId id, double degree, bool calibration = false);
    AltimeterDriveResult setBrightness(uint8_t brightness);

    Altimeter::AltimeterDriveResult moveToHeight(double heightInFeet);

    double currentHeightInFeet();

    String errorName(AltimeterDriveResult result);
    
    // Singleton instance
    static Altimeter* instance;

private:
    AltimeterDriveResult lookForZeroChange(AltimeterAxis axis, int32_t degree, bool targetZeroedState); // Synchronous
    void fetchZeroedState(AltimeterAxis axis); 
    void moveSteps(AltimeterAxis axis, int32_t steps, bool synchron = false);
    void moveDegree(AltimeterAxis axis, int32_t degree, bool synchron = false);
    int32_t calculateShortestPath(AltimeterAxis axis, int32_t targetPosition);
    uint32_t normalizePosition(int32_t position, uint32_t totalSteps);
    String axisName(AltimeterAxis axis);
    String servoName(ServoId id);
    void sendMotorData();
    AltimeterDriveResult nextHomingState(AltimeterAxis axis);
    bool checkAllHomed();
    AltimeterDriveResult lookForZeroChangeNonBlocking(AltimeterAxis axis, bool targetZeroedState);

    bool homingActive = false;
    void runHomingStep();

private:
    struct Config
    {
        uint32_t magic;
        uint16_t version;
        int16_t zeroAdjustDegree[altimeterAxisCount];
        BaroCalibration baro;
    };

    void loadConfig();
    void saveConfig();
    void applyConfigDefaults();

    Config config;

    MCP23017 *mcp;

    Servo* servos[servoCount];
    CheapStepper* axes[altimeterAxisCount];
    HomingPhase homingState[altimeterAxisCount];
    uint32_t zeroEndPosition[altimeterAxisCount];

    uint8_t hundredsPattern = 0;
    uint8_t thousandsPattern = 0;
    uint8_t tenThousandsPattern = 0;

    uint8_t currentPortA = 0;
    uint8_t currentPortB = 0;

    double currentFlagDegree = kServoMaximumDegree[flagServo];
    bool axesCoupled = false;

    bool zeroedState[altimeterAxisCount] = {false, false, false};

    const int32_t DEGREE_FULL_ROTATION = 360L;
};

#endif // ALTIMETER_H