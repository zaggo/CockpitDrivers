#ifndef X25MOTORS_H
#define X25MOTORS_H

#include "Configuration.h"
#include <SwitecX25.h>

class X25Motors {
    public:
        X25Motors();
        ~X25Motors();

        void setPosition(uint8_t motorIndex, float relPos);
        void updateAllX25Steppers();

        void homeAllX25Steppers();
    private:
        SwitecX25* x25Steppers[kX25MotorCount];
};

#endif // X25MOTORS_H