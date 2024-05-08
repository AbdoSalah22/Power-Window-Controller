#include "MOTOR.h"

void MOTOR_init(void){
	GPIO_init(PORTA, PIN2, DIGITAL, OUTPUT);
	GPIO_init(PORTA, PIN3, DIGITAL, OUTPUT);
}

void motorSpin(uint8_t direction){
	if (direction){
		GPIO_setPin(PORTA, PIN2);
		GPIO_clearPin(PORTA, PIN3);
	}
	else{
		GPIO_clearPin(PORTA, PIN2);
		GPIO_setPin(PORTA, PIN3);
	}
}
		
void motorStop(void){
	GPIO_clearPin(PORTA, PIN2);
	GPIO_clearPin(PORTA, PIN3);
}
