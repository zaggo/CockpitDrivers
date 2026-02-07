#ifndef MOTIONACTOR_H
#define MOTIONACTOR_H
#include <Arduino.h>
#include <Kangaroo.h>
#include "Configuration.h"
#include <MotionNodeId.h>

class MotionActor {
    public:
        MotionActor();
        ~MotionActor();

        void home();
        void setDemands(uint16_t demand1, uint16_t demand2);
        void powerDown();

        MotionActorState state;
    private:
        HardwareSerial& kangarooSerial;
        KangarooSerial  K;
        KangarooChannel actor1;
        KangarooChannel actor2; 

        int32_t minPosition1 = 0;
        int32_t maxPosition1 = 0;
        int32_t minPosition2 = 0;
        int32_t maxPosition2 = 0;
};

#endif // MOTIONACTOR_H