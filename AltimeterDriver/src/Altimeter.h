#ifndef ALTIMETER_H
#define ALTIMETER_H
#include <Arduino.h>
#include <Servo.h>
#include <MCP23017.h>
#include "CheapStepper.h"
#include "Configuration.h"

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
    float fetchPressureRatio();

    void loop();
    void stopAllAxes();
    void offAllAxes();

    AltimeterDriveResult homeAllAxis(); // Synchronous
    AltimeterDriveResult homeAxis(AltimeterAxis axis); // Synchronous
    AltimeterDriveResult moveToDegree(double hundredDegree, double thousandDegree, double tenThousandDegree, double flagDegree); // Ansyc
    AltimeterDriveResult moveToDegree(AltimeterAxis axis, double degree);
    AltimeterDriveResult moveServo(ServoId id, double degree, bool calibration = false);

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

private:
    MCP23017 *mcp;

    Servo* servos[servoCount];
    CheapStepper* axes[altimeterAxisCount];
    HomingPhase homingState[altimeterAxisCount];
    uint32_t zeroEndPosition[altimeterAxisCount];

    uint8_t hundredsPattern = 0;
    uint8_t thousandsPattern = 0;

    uint8_t currentPortA = 0;
    uint8_t currentPortB = 0;

    double currentFlagDegree = kServoMaximumDegree[flagServo];
    bool axesCoupled = false;

    bool zeroedState[altimeterAxisCount] = {false, false, false};

    const int32_t DEGREE_FULL_ROTATION = 360L;
};

#endif // ALTIMETER_H