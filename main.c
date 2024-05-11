#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include "queue.h"
#include "semphr.h"
#include "MOTOR.h"
#include "LED.h"
#include "delay.h"

#define PortD_IRQn 3
#define PortF_IRQn 30

#define driverUpButton (GPIO_readPin(PORTD, PIN2))
#define driverDownButton (GPIO_readPin(PORTD, PIN3))
#define passengerUpButton (GPIO_readPin(PORTD, PIN6))
#define passengerDownButton (GPIO_readPin(PORTD, PIN7))

#define windowLock (GPIO_readPin(PORTA, PIN4))

#define upLimit (GPIO_readPin(PORTA, PIN6))
#define downLimit (GPIO_readPin(PORTA, PIN7))


xSemaphoreHandle driverUpSemaphore;
xSemaphoreHandle driverDownSemaphore;
xSemaphoreHandle passengerUpSemaphore;
xSemaphoreHandle passengerDownSemaphore;
xSemaphoreHandle jamSemaphore;

xSemaphoreHandle motorMutex;

volatile long startTime;
volatile long endTime;


void driverUpTask(void *pvParameters){
	xSemaphoreTake(driverUpSemaphore, 0);
	for(;;){
		xSemaphoreTake( driverUpSemaphore, portMAX_DELAY );
		xSemaphoreTake( motorMutex, portMAX_DELAY);
		
		if(!driverUpButton && upLimit){
			startTime = xTaskGetTickCount();
			while(!driverUpButton && upLimit){
				redOn();
				motorSpin(FORWARD);
			}
			endTime = xTaskGetTickCount();
			
			if((endTime - startTime < 100)){
				while(upLimit){
					motorSpin(FORWARD);
					blueOn();
				}
			}
		}
		motorStop();
		whiteOff();
		xSemaphoreGive(motorMutex);
		delayUs(100); // Short NOP to handle debouncing
	}
}


void driverDownTask(void *pvParameters){
	xSemaphoreTake(driverDownSemaphore, 0);
	for(;;){
		xSemaphoreTake( driverDownSemaphore, portMAX_DELAY );
		xSemaphoreTake( motorMutex, portMAX_DELAY);
		
		if(!driverDownButton && downLimit){
			startTime = xTaskGetTickCount();
			while(!driverDownButton && downLimit){
				redOn();
				motorSpin(BACKWARD);
			}
			endTime = xTaskGetTickCount();
			
			if((endTime - startTime < 100)){
				while(downLimit){
					motorSpin(BACKWARD);
					blueOn();
				}
			}
		}
		motorStop();
		whiteOff();
		xSemaphoreGive(motorMutex);
		delayUs(100); // Short NOP to handle debouncing
	}
}


void passengerUpTask(void *pvParameters){
	xSemaphoreTake(passengerUpSemaphore, 0);
	for(;;){
		xSemaphoreTake( passengerUpSemaphore, portMAX_DELAY );
		xSemaphoreTake( motorMutex, portMAX_DELAY);
		
		if(!passengerUpButton && upLimit && windowLock){
			startTime = xTaskGetTickCount();
			while(!passengerUpButton && upLimit && windowLock){
				redOn();
				motorSpin(FORWARD);
			}
			endTime = xTaskGetTickCount();
			
			if((endTime - startTime < 100)){
				while(upLimit && windowLock){
					motorSpin(FORWARD);
					blueOn();
				}
			}
		}
		motorStop();
		whiteOff();
		xSemaphoreGive(motorMutex);
		delayUs(100); // Short NOP to handle debouncing
	}
}


void passengerDownTask(void *pvParameters){
	xSemaphoreTake(passengerDownSemaphore, 0);
	for(;;){
		xSemaphoreTake( passengerDownSemaphore, portMAX_DELAY );
		xSemaphoreTake( motorMutex, portMAX_DELAY);
		
		if(!passengerDownButton && downLimit && windowLock){
			startTime = xTaskGetTickCount();
			while(!passengerDownButton && downLimit && windowLock){
				redOn();
				motorSpin(BACKWARD);
			}
			endTime = xTaskGetTickCount();
			
			if((endTime - startTime < 100)){
				while(downLimit && windowLock){
					motorSpin(BACKWARD);
					blueOn();
				}
			}
		}
		motorStop();
		whiteOff();
		xSemaphoreGive(motorMutex);
		delayUs(100); // Short NOP to handle debouncing
	}
}

// give mutex after counting time to enable jam and other cuts to window
void jamTask(void *pvParameters){
	xSemaphoreTake(driverUpSemaphore, 0);
	for(;;){
		xSemaphoreTake( jamSemaphore, portMAX_DELAY );
		xSemaphoreTake( motorMutex, portMAX_DELAY);
		
		greenOn();
		motorSpin(BACKWARD);
		delayMs(500);
		
		motorStop();
		whiteOff();
		xSemaphoreGive(motorMutex);
		delayMs(100); // replace with vTaskDelay?
	}
}


void PortA_Init(void){
		// Motor on pins A2, A3
		MOTOR_init();
	
		// Window Lock on pin A4 - Limit Switches on A6, A7
		SYSCTL->RCGCGPIO |= 0x00000001;
		GPIOA->CR |= 0x10;
		GPIOA->DIR &= ~0x10;
		GPIOA->PUR |= 0x10;
		GPIOA->DEN |= 0x10;
	
		GPIOA->CR |= 0xC0;
		GPIOA->DIR &= ~0xC0;
		GPIOA->PUR |= 0xC0;
		GPIOA->DEN |= 0xC0;
	
		GPIOA->DATA = 0x00;
}



// Initialize the hardware of Port D for pins 2, 3, 6, 7 with interrupts
void PortD_Init(void) {
    SYSCTL->RCGCGPIO |= 0x00000008; // Enable clock for Port D
    GPIO_PORTD_LOCK_R = GPIO_LOCK_KEY; // Unlock Port D
    GPIO_PORTD_CR_R |= 0xCC; // Allow changes to PD2, PD3, PD6, PD7
    GPIO_PORTD_AMSEL_R = 0x00; // Disable analog function
    GPIO_PORTD_PCTL_R = 0x00000000; // GPIO clear bit PCTL
    GPIO_PORTD_DIR_R &= ~0xCC; // PD2, PD3, PD6, PD7 input
    GPIO_PORTD_AFSEL_R = 0x00; // No alternate function
    GPIO_PORTD_PUR_R |= 0xCC; // Enable pull-up resistors on PD2, PD3, PD6, PD7
    GPIO_PORTD_DEN_R |= 0xCC; // Enable digital pins PD2, PD3, PD6, PD7
    
    // Setup interrupts for Port D pins
    GPIO_PORTD_ICR_R = 0xCC; // Clear any previous interrupts
    GPIO_PORTD_IM_R |= 0xCC; // Unmask interrupts for PD2, PD3, PD6, PD7
    GPIO_PORTD_IS_R &= ~0xCC; // Make bits PD2, PD3, PD6, PD7 edge-sensitive
    GPIO_PORTD_IBE_R &= ~0xCC; // Disable both edges triggering
    GPIO_PORTD_IEV_R &= ~0xCC; // Sense on falling edge
    NVIC_EnableIRQ(PortD_IRQn); // Enable the Interrupt for Port D in NVIC
    NVIC_PRI0_R = (NVIC_PRI0_R & 0x00FFFFFF) | 0xA0000000; // Set priority to 5 for GPIOD interrupt
}


// Port D ISR Handler for driver and passenger controls
void GPIOD_Handler(void) {
		//portBASE_TYPE xHigherPriorityTaskWoken = configMAX_SYSCALL_INTERRUPT_PRIORITY; ??
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

    // Check if pin 2 interrupt occurred
    if (GPIO_PORTD_RIS_R & (1 << 2)) {
				xSemaphoreGiveFromISR( driverUpSemaphore, &xHigherPriorityTaskWoken );
        GPIO_PORTD_ICR_R |= (1 << 2); // Clear interrupt flag for pin 2
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    
    // Check if pin 3 interrupt occurred
    else if (GPIO_PORTD_RIS_R & (1 << 3)) {
				xSemaphoreGiveFromISR( driverDownSemaphore, &xHigherPriorityTaskWoken );
        GPIO_PORTD_ICR_R |= (1 << 3); // Clear interrupt flag for pin 3
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    
    // Check if pin 6 interrupt occurred
    else if (GPIO_PORTD_RIS_R & (1 << 6)) {
				xSemaphoreGiveFromISR( passengerUpSemaphore, &xHigherPriorityTaskWoken );
        GPIO_PORTD_ICR_R |= (1 << 6); // Clear interrupt flag for pin 6
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
    
    // Check if pin 7 interrupt occurred
    else if (GPIO_PORTD_RIS_R & (1 << 7)) {
				xSemaphoreGiveFromISR( passengerDownSemaphore, &xHigherPriorityTaskWoken );
        GPIO_PORTD_ICR_R |= (1 << 7); // Clear interrupt flag for pin 7
				portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
    }
		
		// Debugging
		else{
			whiteOn();
		}
}


//Initialize the hardware of Port-F
void PortF_Init(void){
		SYSCTL->RCGCGPIO |= 0x00000020; // 1) F clock
		GPIOF->LOCK = 0x4C4F434B; // 2) unlock PortF PF0
		GPIOF->CR = 0x11; // allow changes to PF4-0
		GPIOF->AMSEL= 0x00; // 3) disable analog function
		GPIOF->PCTL = 0x00000000; // 4) GPIO clear bit PCTL
		GPIOF->DIR = 0x0E; // 5) PF4,PF0 input, PF3,PF2,PF1 output
		GPIOF->AFSEL = 0x00; // 6) no alternate function
		GPIOF->PUR = 0x11; // enable pullup resistors on PF4,PF0
		GPIOF->DEN = 0x11; // 7) enable digital pins PF4-PF0
		GPIOF->DATA = 0x00;
	
		// Setup the interrupt on PortF
		GPIOF->ICR = 0x11; // Clear any Previous Interrupt
		GPIOF->IM |=0x11; // Unmask the interrupts for PF0 and PF4
		GPIOF->IS |= 0x11; // Make bits PF0 and PF4 level sensitive
		GPIOF->IEV &= ~0x11; // Sense on Low Level
		NVIC_EnableIRQ(PortF_IRQn); // Enable the Interrupt for PortF in NVIC
	  NVIC_PRI7_R = (NVIC_PRI7_R & 0xFF00FFFF) | 0x00600000;  // Set priority to 3 for IRQ30
}


void GPIOF_Handler(void){
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	
	// Give semaphore to jam task
	xSemaphoreGiveFromISR( jamSemaphore, &xHigherPriorityTaskWoken );
	
	// Clear the interrupt flag of PORTF
  GPIOF->ICR = 0x11;
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}



int main(void) {
	
	PortA_Init();
	PortD_Init();
	PortF_Init();
	LED_init();
	
	
	driverUpSemaphore = xSemaphoreCreateBinary();
	driverDownSemaphore = xSemaphoreCreateBinary();
	passengerUpSemaphore = xSemaphoreCreateBinary();
	passengerDownSemaphore = xSemaphoreCreateBinary();
	jamSemaphore = xSemaphoreCreateBinary();
	
	motorMutex = xSemaphoreCreateMutex();


	xTaskCreate(driverUpTask, "driverUp", 40, NULL, 2, NULL);
	xTaskCreate(driverDownTask, "driverDown", 40, NULL, 2, NULL);
	xTaskCreate(passengerUpTask, "passengerUp", 40, NULL, 2, NULL);
	xTaskCreate(passengerDownTask, "passengerDown", 40, NULL, 2, NULL);

	xTaskCreate(jamTask, "jam", 40, NULL, 3, NULL);

	vTaskStartScheduler();

	for(;;){
		;
	}
}