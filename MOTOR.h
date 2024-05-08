#ifndef MOTOR_H
#define MOTOR_H

#include "GPIO.h"

#define FORWARD 1
#define BACKWARD 0


void MOTOR_init(void);
void motorSpin(uint8_t direction);
void motorStop(void);


#endif //MOTOR_H