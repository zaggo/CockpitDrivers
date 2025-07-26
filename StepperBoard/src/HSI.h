#ifndef HSI_H
#define HSI_H

#include "Arduino.h"
#include "CheapStepper.h"
#include "Configuration.h"
#include <Servo.h>

class HSI {
    public:
        enum ServoId {
        vorServo = 0,
        fromToServo,
        gsServo,
                
        servoCount
        };
    
        enum FromTo {
            noNav = 180,
            from = 90,
            to = 0
        };

        enum HSIDriveResult {
        success = 0,
        homingError,
        notHomed,
        invalidId
        };

        HSI();
        ~HSI();

        HSIDriveResult homeAllAxis(); // Synchronous
        HSIDriveResult moveToDegree(double vorDegree, double compassDegree, double bugDegree, double vorOffset, FromTo fromTo, double gsOffset); // Ansyc
        HSIDriveResult moveToDegree(HSIAxis axis, double degree);
        HSIDriveResult moveServo(ServoId id, double degree);

        void runAllAxes();
        void stopAllAxes();
        void offAllAxes();

        // Singleton instance
        static HSI* instance;

    private:

        HSIDriveResult homeAxis(HSIAxis axis); // Synchronous
        HSIDriveResult lookForZeroChange(HSIAxis axis, int32_t degree, bool targetZeroedState); // Synchronous
        void moveSteps(HSIAxis axis, int16_t steps, bool synchron = false);
        void moveDegree(HSIAxis axis, int16_t degree, bool synchron = false);
        bool getZeroedState(HSIAxis axis);
        int32_t calculateShortestPath(HSIAxis axis, int32_t targetPosition);
        uint32_t normalizePosition(int32_t position, uint32_t totalSteps);

        String axisName(HSIAxis axis);

        Servo* servos[ServoId::servoCount];
        CheapStepper* axes[HSIAxis::hsiAxisCount];

        // An array of the active axis
        static const uint8_t activeAxesCount = 1;
        const HSIAxis activeAxes[activeAxesCount] = { bug };
        
        volatile bool zeroedState[HSIAxis::hsiAxisCount] = { false };
        bool isHomed = false;

        // static void ISR_VorPinChange();
        // static void ISR_CompassPinChange();
        // static void ISR_BugPinChange();

        const int32_t DEGREE_FULL_ROTATION = 360L;

        // Configuration methods
        void initializeStepper(HSIAxis axis);
};
#endif // HSI_H