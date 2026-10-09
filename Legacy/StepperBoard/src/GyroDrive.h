
#ifndef GYRODRIVE_H
#define GYRODRIVE_H

#include "Arduino.h"
#include "CheapStepper.h"

class GyroDrive
{
  public:
    enum Axis {
      roll = 0,
      pitch,

      count
    };
  
    enum GyroDriveResult {
      success = 0,
      homingError,
      notHomed
    };
 
    GyroDrive();
    ~GyroDrive();

    GyroDriveResult homeAllAxis(); // Synchronous
    GyroDriveResult moveToDegree(double rollDegree, double pitchDegree); // Ansyc
    void moveSteps(int32_t numStepsRoll, int32_t numStepsPitch, bool correctPitchForRoll, bool inhibitSpeedChange = false); // Ansyc

    void runAllAxes();
    void stopAllAxes();
    void offAllAxes();

    // Singleton instance
    static GyroDrive* instance;

  private:  

    GyroDriveResult homeRollAxis(); // Synchronous
    GyroDriveResult homePitchAxis(); // Synchronous
    GyroDriveResult lookForZeroChange(Axis axis, int32_t degree, bool targetZeroedState, bool inhibitSpeedChange = false); // Synchronous

    void moveDegree(int16_t rollDegree, int16_t pitchDegree, bool inhibitSpeedChange = false); // Ansyc
    int32_t calculateShortestPath(uint32_t currentPosition, int32_t targetPosition, uint32_t totalSteps);
    uint32_t convertPosition(Axis fromAxis, uint32_t position);
    uint32_t normalizePosition(int32_t position, uint32_t totalSteps);

    CheapStepper* axes[Axis::count];
    volatile bool rollZeroedState = false;
    volatile bool pitchZeroedState = false;
    bool isHomed = false;

    static void ISR_RollPinChange();
    static void ISR_PitchPinChange();

    const int32_t DEGREE_FULL_ROTATION = 360L;

    // Configuration methods
    void initializeStepper(Axis axis, uint8_t pin1, uint8_t pin2, uint8_t pin3, uint8_t pin4, uint32_t totalSteps, double maxRpm);
  };
#endif
