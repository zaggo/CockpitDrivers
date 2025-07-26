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
        invalidId
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
    HSIDriveResult moveToDegree(double cdiDegree, double compassDegree, double hdgDegree, double vorOffset, FromTo fromTo, double vsiOffset); // Ansyc
    HSIDriveResult moveToDegree(HSIAxis axis, double degree);
    HSIDriveResult moveServo(ServoId id, double degree);

    int16_t getCdiEncoder() { return cdiEncoder; }
    int16_t getCompEncoder() { return compEncoder; }
    bool getCdiEncoderButtonState() { return cdiEncoderButtonState; }
    bool getCompEncoderButtonState() { return compEncoderButtonState; }

    // Singleton instance
    static HSI* instance;
    volatile uint8_t cdiEncoderState;
    volatile int16_t currentCdiEncoder = 0;
    volatile uint8_t compEncoderState;
    volatile int16_t currentCompEncoder = 0;
    volatile bool currentCdiEncoderButtonState;
    volatile bool currentCompEncoderButtonState;

private:
    HSIDriveResult lookForZeroChange(HSIAxis axis, int32_t degree, bool targetZeroedState); // Synchronous
    void fetchZeroedState(HSIAxis axis); 
    void moveSteps(HSIAxis axis, int16_t steps, bool synchron = false);
    void moveDegree(HSIAxis axis, int16_t degree, bool synchron = false);
    int32_t calculateShortestPath(HSIAxis axis, int32_t targetPosition);
    uint32_t normalizePosition(int32_t position, uint32_t totalSteps);
    String axisName(HSIAxis axis);
    String servoName(ServoId id);
    void sendMotorData();
private:
    MCP23017 *mcp;

    Servo* servos[servoCount];
    CheapStepper* axes[hsiAxisCount];

    uint8_t cdiPattern = 0;
    uint8_t hdgPattern = 0;
    uint8_t compPattern = 0;

    uint8_t currentPortA = 0;
    uint8_t currentPortB = 0;

    int16_t cdiEncoder = 0;
    int16_t compEncoder = 0;
    bool cdiEncoderButtonState = false;
    bool compEncoderButtonState = false;
    uint32_t lastCdiDebounceTime = 0;
    uint32_t lastCompDebounceTime = 0;

    bool zeroedState[hsiAxisCount] = {false, false, false};

    const int32_t DEGREE_FULL_ROTATION = 360L;
};

#endif // HSI_H