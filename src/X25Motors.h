#ifndef X25MOTORS_H
#define X25MOTORS_H

#include "Configuration.h"
#include <SwitecX25.h>

extern SwitecX25* x25Steppers[kX25MotorCount];
void initX25Steppers();
void updateAllX25Steppers();

#endif // X25MOTORS_H