
//#define and needed Libraries

#include <stdbool.h>
#include <stdint.h>
#include <FreeRTOS.h>
#include <task.h>
#include <FreeRTOSConfig.h>
#include "tm4c123gh6pm.h"
#include "timers.h"
#include "lcd.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "semphr.h"
#include "inc/hw_types.h"
#include "driverlib/interrupt.h"
#include "inc/hw_gpio.h"


#define DEBOUNCE_DELAY_MS 50
#define LONG_PRESS_DELAY_MS 1000

//strings to be printed
static const char *pctextforlcd1 = "driver up auto";
static const char *pctextforlcd2 = "driver up manu";
static const char *pctextforlcd3 = "driver down auto";
static const char *pctextforlcd4 = "driver down manu";
static const char *pctextforlcd5 = "pass up auto";
static const char *pctextforlcd6 = "pass up manu";
static const char *pctextforlcd7 = "pass down auto";
static const char *pctextforlcd8 = "pass down manu";

//global variables needed to check for jamming , locking window , window limits 
static volatile int jamDetected = 0;
static volatile int windowLock = 0;
static volatile bool upperLimitReached = false;
static volatile bool lowerLimitReached = false;

static void vlcdWrite(void *pvParameters);
void portCinit();
void portEinit();
void buttonTask(void *pvParameters);
void drivUpAuto(void *pvParameters);
void drivDownAuto(void *pvParameters);
void drivUpManu(void *pvParameters);
void drivDownManu(void *pvParameters);
void passUpAuto(void *pvParameters);
void passDownAuto(void *pvParameters);
void passUpManu(void *pvParameters);
void passDownManu(void *pvParameters);
void setUpperLimit(void *pvParameters);
void setLowerLimit(void *pvParameters);
void setWindowLock(void *pvParameters);
void setJamDetected(void *pvParameters);


//handles for all tasks
TaskHandle_t xdrivUpAutoHandle;
TaskHandle_t xdrivUpManuHandle;
TaskHandle_t xdrivDownAutoHandle;
TaskHandle_t xdrivDownManuHandle;
TaskHandle_t xpassUpAutoHandle;
TaskHandle_t xpassUpManuHandle;
TaskHandle_t xpassDownAutoHandle;
TaskHandle_t xpassDownManuHandle;


//semaphores
// Define the binary semaphore
SemaphoreHandle_t xupperLimitReachedSemaphore;
SemaphoreHandle_t xlowerLimitReachedSemaphore;
SemaphoreHandle_t xwindowLockSemaphore;
SemaphoreHandle_t xjamDetectedSemaphore;


//queue
QueueHandle_t xQueue;

//task handle passed between tasks
TaskHandle_t xTaskHandlePassed;

int main(){

	xQueue = xQueueCreate(1, sizeof(TaskHandle_t));
		// Create the binary semaphore
//	vSemaphoreCreateBinary(xupperLimitReachedSemaphore);
  xupperLimitReachedSemaphore = xSemaphoreCreateBinary();
//	vSemaphoreCreateBinary(xlowerLimitReachedSemaphore);
	xlowerLimitReachedSemaphore = xSemaphoreCreateBinary();
//	vSemaphoreCreateBinary(xwindowLockSemaphore);
	xwindowLockSemaphore = xSemaphoreCreateBinary();
//	vSemaphoreCreateBinary(xjamDetectedSemaphore);
	xjamDetectedSemaphore = xSemaphoreCreateBinary();
	portCinit();
	portEinit();
	LCD_init();
	LCD_Clear();
	

	
	// Create the task
  xTaskCreate(setUpperLimit, "setupper", 128, NULL, 4, NULL);
	xTaskCreate(setLowerLimit, "setlower", 128, NULL, 4, NULL);
	xTaskCreate(setWindowLock, "setwindowlock", 128, NULL, 4, NULL);
	xTaskCreate(setJamDetected, "setjamdetected", 128, NULL, 4, NULL);

	
  // Create the button task
  xTaskCreate( buttonTask, "Button Task", 128, NULL, 1, NULL );

	vTaskStartScheduler();
		
}



//This task will be the lowest priority task that checks every button (up & down) of passenger and driver 
void buttonTask(void *pvParameters)
{	
    // Define variables to keep track of button states and times
		TickType_t xLastWakeTime;
    const TickType_t xDelay = pdMS_TO_TICKS(1);
    xLastWakeTime = xTaskGetTickCount();
	  int pc4_pressed = 0;
    int pc5_pressed = 0;
    int pc4_long_pressed = 0;
    int pc5_long_pressed = 0;
		int pc6_pressed = 0;
    int pc7_pressed = 0;
    int pc6_long_pressed = 0;
    int pc7_long_pressed = 0;
		
		while (1){
        // Check if PC4 button is pressed (driver up)
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
            if (!pc4_pressed) {
                pc4_pressed = 1;
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
                if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
                    // PC4 button is still pressed
                    xLastWakeTime = xTaskGetTickCount();
                    while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        vTaskDelay(xDelay);
                    }
                    if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
												// PA0 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd2);
												xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 2, &xdrivUpManuHandle );
												LCD_Clear();
												//pc4_long_pressed = 1;
											
										} 
										else {
											// PA0 button has been pressed and released almost immediately (auto)
											//vlcdWrite((void*)pctextforlcd1);
											xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 2, &xdrivUpAutoHandle );
											LCD_Clear();
                    }
                }
                pc4_pressed = 0;
            }
						
        }	
				pc4_pressed = 0;	

				
        // Check if PC5 button is pressed  (driver down)
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
            if (!pc5_pressed) {
                pc5_pressed = 1;
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
                if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) {
                    // PC5 button is still pressed
                    xLastWakeTime = xTaskGetTickCount();
                    while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        vTaskDelay(xDelay);
                    }
                    if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        // PC5 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd4);
												xTaskCreate( drivDownManu, "driverDownManu", 128, NULL, 2, &xdrivDownManuHandle );
												LCD_Clear();
                        //pc5_long_pressed = 1;
											
                    } else {
                        // PA1 button has been pressed and released almost immediately (auto)
                        //vlcdWrite((void*)pctextforlcd3);
												xTaskCreate( drivDownAuto, "driverDownAuto", 128, NULL, 2, &xdrivDownAutoHandle );
												LCD_Clear();
                    }
                }
                pc5_pressed = 0;
            }
						
        }	
				pc5_pressed = 0;	
				// Check if PC6 button is pressed (passenger up)
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) {
            if (!pc6_pressed) {
                pc6_pressed = 1;
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
                if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) {
                    // PC6 button is still pressed
                    xLastWakeTime = xTaskGetTickCount();
                    while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        vTaskDelay(xDelay);
                    }
                    if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        // PA0 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd6);
												xTaskCreate( passUpManu, "passUpManu", 128, NULL, 2, &xpassUpManuHandle );
												LCD_Clear();
                        pc6_long_pressed = 1;
											
                    } else {
                        // PA0 button has been pressed and released almost immediately (auto)
                        //vlcdWrite((void*)pctextforlcd5);
												xTaskCreate( passUpAuto, "passUpAuto", 128, NULL, 2, &xpassUpAutoHandle );
												LCD_Clear();											
                    }
                }
                pc6_pressed = 0;
            }
						
        }	
				pc6_pressed = 0;	

        // Check if PC7 button is pressed (passenger down)
        if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) {
            if (!pc7_pressed) {
                pc7_pressed = 1;
                vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
                if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) {
                    // PC7 button is still pressed
                    xLastWakeTime = xTaskGetTickCount();
                    while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        vTaskDelay(xDelay);
                    }
                    if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
                        // PC5 button has been pressed for 2 seconds (manu)
												//vlcdWrite((void*)pctextforlcd8);
												xTaskCreate( passDownManu, "passDownAuto", 128, NULL, 2, &xpassDownManuHandle );
												LCD_Clear();
                        pc7_long_pressed = 1;
											
                    } else {
                        // PA1 button has been pressed and released almost immediately (auto)
                        //vlcdWrite((void*)pctextforlcd7);
												xTaskCreate( passDownAuto, "passDownManu", 128, NULL, 2, &xpassDownAutoHandle );
												LCD_Clear();
                    }
                }
                pc7_pressed = 0;
            }
						
        }	
				pc7_pressed = 0;					
		}

}

void drivUpAuto(void *pvParameters)
{
	LCD_Clear();
	TaskHandle_t xReceivedHandle;
	if (xQueueReceive(xQueue, &xReceivedHandle, 0) == pdTRUE) {
			// Perform action if a handle is received
			vTaskDelete(xReceivedHandle);
	}
	vlcdWrite((void*)pctextforlcd1);
	while((!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//turn motor uppp
		lowerLimitReached = false;
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivUpAutoHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xdrivUpAutoHandle);
	}
	if(upperLimitReached){
		LCD_PrintLn(1,"Upper limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivUpAutoHandle);
	}


	vTaskDelete(xdrivUpAutoHandle);
}



void drivUpManu(void *pvParameters)
{	
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd2);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && (!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//turn motor uppp
		lowerLimitReached = false;
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)){
		LCD_PrintLn(1,"button release");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivUpManuHandle);
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivUpManuHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xdrivUpManuHandle);
	}
	if(upperLimitReached){
		LCD_PrintLn(1,"Upper limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivUpManuHandle);
	}

	vTaskDelete(xdrivUpManuHandle);
}





void drivDownAuto(void *pvParameters)
{	
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd3);
	while((!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){//ask if jam detected needed here??
		//turn motor downnn
		upperLimitReached = false;
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivDownAutoHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xdrivDownAutoHandle);
	}
	if(lowerLimitReached){
		LCD_PrintLn(1,"Lower limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivDownAutoHandle);
	}

	vTaskDelete(xdrivDownAutoHandle);
}



void drivDownManu(void *pvParameters)
{
	LCD_Clear();	
	vlcdWrite((void*)pctextforlcd4);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)) && (!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//turn motor downnn
		upperLimitReached = false;
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_5)){
	LCD_PrintLn(1,"button release");
	//vTaskDelay(1000/portTICK_RATE_MS);
	vTaskDelete(xdrivDownManuHandle);
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivDownManuHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xdrivDownManuHandle);
	}
	if(lowerLimitReached){
		LCD_PrintLn(1,"Lower limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xdrivDownManuHandle);
	}

	vTaskDelete(xdrivDownManuHandle);
}




void passUpAuto(void *pvParameters)
{
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd5);
	while((!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//turn motor uppp
		lowerLimitReached = false;
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassUpAutoHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xpassUpAutoHandle);
	}
	if(upperLimitReached){
		LCD_PrintLn(1,"Upper limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassUpAutoHandle);
	}


	vTaskDelete(xpassUpAutoHandle);
}




void passUpManu(void *pvParameters)
{
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd6);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)) && (!upperLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//turn motor uppp
		lowerLimitReached = false;
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_6)){
		LCD_PrintLn(1,"button release");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassUpManuHandle);
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassUpManuHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xpassUpManuHandle);
	}
	if(upperLimitReached){
		LCD_PrintLn(1,"Upper limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassUpManuHandle);
	}
	vTaskDelete(xpassUpManuHandle);
}




void passDownAuto(void *pvParameters)
{
		// Define variables to keep track of button states and times
	TickType_t xLastWakeTime;
	const TickType_t xDelay = pdMS_TO_TICKS(1);
	xLastWakeTime = xTaskGetTickCount();
	int pc4_pressed = 0;
	int pc5_pressed = 0;
	int pc4_long_pressed = 0;
	int pc5_long_pressed = 0;
	LCD_Clear();
	vlcdWrite((void*)pctextforlcd7);
	while((!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){//ask if jam detected needed here??
		//turn motor downnn
		upperLimitReached = false;
		
		//read driver buttons
		if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
				if (!pc4_pressed) {
						pc4_pressed = 1;
						vTaskDelay(pdMS_TO_TICKS(DEBOUNCE_DELAY_MS));
						if (!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) {
								// PC4 button is still pressed
								xLastWakeTime = xTaskGetTickCount();
								while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) < pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
										vTaskDelay(xDelay);
								}
								if ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_4)) && ((xTaskGetTickCount() - xLastWakeTime) >= pdMS_TO_TICKS(LONG_PRESS_DELAY_MS))) {
										// PA0 button has been pressed for 2 seconds (manu)
										//vlcdWrite((void*)pctextforlcd2);
										LCD_Clear();
										xTaskCreate( drivUpManu, "driverUpManu", 128, NULL, 2, &xdrivUpManuHandle );
										LCD_Clear();
										//pc4_long_pressed = 1;
									
								} 
								else {
									// PA0 button has been pressed and released almost immediately (auto)
									//vlcdWrite((void*)pctextforlcd1);
									LCD_Clear();
									xTaskHandlePassed = xTaskGetCurrentTaskHandle();
									xQueueSend(xQueue, &xTaskHandlePassed, portMAX_DELAY);
									xTaskCreate( drivUpAuto, "driverUpAuto", 128, NULL, 3, &xdrivUpAutoHandle );
									LCD_Clear();
								}
						}
						pc4_pressed = 0;
				}
						
     }			
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassDownAutoHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xpassDownAutoHandle);
	}
	if(lowerLimitReached){
		LCD_PrintLn(1,"Lower limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassDownAutoHandle);
	}

	vTaskDelete(xpassDownAutoHandle);
}




void passDownManu(void *pvParameters)
{
	LCD_Clear();	
	vlcdWrite((void*)pctextforlcd8);
	while ((!GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)) && (!lowerLimitReached) && (jamDetected == 0) && (windowLock == 0)){
		//turn motor downnn
		upperLimitReached = false;		
	}
	if(GPIOPinRead(GPIO_PORTC_BASE, GPIO_PIN_7)){
	LCD_PrintLn(1,"button release");
	//vTaskDelay(1000/portTICK_RATE_MS);
	vTaskDelete(xpassDownManuHandle);
	}
	if(windowLock == 1){
		LCD_PrintLn(1,"Window lock");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassDownManuHandle);
	}
	if(jamDetected == 1){
		LCD_PrintLn(1,"Jam detected");
		//implement jam detection protocol
		//vTaskDelay(1000/portTICK_RATE_MS);
		jamDetected = 0;
		vTaskDelete(xpassDownManuHandle);
	}
	if(lowerLimitReached){
		LCD_PrintLn(1,"Lower limit");
		//vTaskDelay(1000/portTICK_RATE_MS);
		vTaskDelete(xpassDownManuHandle);
	}
	vTaskDelete(xpassDownManuHandle);
}







// semaphore tasks

void setUpperLimit(void *pvParameters)
{
	xSemaphoreTake(xupperLimitReachedSemaphore, 0);
  // Wait for the semaphore
	while(1){
  xSemaphoreTake(xupperLimitReachedSemaphore, portMAX_DELAY);
	upperLimitReached = true;
	}
  // Do something when the semaphore is given
}

void setLowerLimit(void *pvParameters)
{
  // Wait for the semaphore
  xSemaphoreTake(xlowerLimitReachedSemaphore, 0);
	while(1){
  xSemaphoreTake(xlowerLimitReachedSemaphore, portMAX_DELAY);
	lowerLimitReached = true;
	}
  // Do something when the semaphore is given
}

void setWindowLock(void *pvParameters)
{
  // Wait for the semaphore
	xSemaphoreTake(xwindowLockSemaphore, 0);
	while(1){
		xSemaphoreTake(xwindowLockSemaphore, portMAX_DELAY);
		if (windowLock == 1){
				windowLock = 0;
		}
		else 
			windowLock = 1;
}
  // Do something when the semaphore is given
}

void setJamDetected(void *pvParameters)
{
  // Wait for the semaphore
	xSemaphoreTake(xjamDetectedSemaphore, 0);
	while(1){
		xSemaphoreTake(xjamDetectedSemaphore, portMAX_DELAY);
		jamDetected = 1;
	}
  // Do something when the semaphore is given
}


static void vlcdWrite(void *pvParameters){
	
	char *pclcdprint = (char*)pvParameters;

	//vTaskDelay(1000/portTICK_RATE_MS);
	LCD_PrintLn(0,pclcdprint);
	//vTaskDelay(1000/portTICK_RATE_MS);
	//LCD_Clear();

}









void PortE_ISR_Handler(void)
{
    uint32_t interrupt_status = GPIOIntStatus(GPIO_PORTE_BASE, true); // Get the interrupt status of the GPIO port E
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    if(interrupt_status & GPIO_PIN_1) // Check if Pin 1 caused the interrupt
    {
        // Pin 1 was pressed (Upper Limit Reached)
				xSemaphoreGiveFromISR(xupperLimitReachedSemaphore, &xHigherPriorityTaskWoken);
        //upperLimitReached = true;
    }

    if(interrupt_status & GPIO_PIN_2) // Check if Pin 2 caused the interrupt
    {
        // Pin 2 was pressed (lower Limit reached)
				xSemaphoreGiveFromISR(xlowerLimitReachedSemaphore, &xHigherPriorityTaskWoken);
				//lowerLimitReached = true;
    }

    if(interrupt_status & GPIO_PIN_3) // Check if Pin 3 caused the interrupt
    {
        // Pin 3 was pressed (window Lock Engaged)
				xSemaphoreGiveFromISR(xwindowLockSemaphore, &xHigherPriorityTaskWoken);
//				if (windowLock == 1){
//						windowLock = 0;
//				}
//				else 
//					windowLock = 1;
    }

    if(interrupt_status & GPIO_PIN_4) // Check if Pin 4 caused the interrupt
    {
        // Pin 4 was pressed (jamming Detected)
				xSemaphoreGiveFromISR(xjamDetectedSemaphore, &xHigherPriorityTaskWoken);
				//jamDetected = 1;
    }
		GPIOIntClear(GPIO_PORTE_BASE, interrupt_status); // Clear the interrupt status of the GPIO port E
		portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}




void portCinit(){
	

		// Enable the GPIO ports
    // Set up GPIO pins for buttons PC4 and PC5
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
//    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7);
  //  GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_0);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
  //  GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

}



void portEinit()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); // Enable the GPIO port E

    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4); // Set Pins 1 to 4 as inputs
    GPIOPadConfigSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // Enable pull-up resistors on Pins 1 to 4

    GPIOIntTypeSet(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4, GPIO_FALLING_EDGE); // Set Pins 1 to 4 as interrupts on falling edge
    GPIOIntEnable(GPIO_PORTE_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4); // Enable interrupts for Pins 1 to 4
	  // Register the ISR in the vector table
    IntRegister(INT_GPIOE, PortE_ISR_Handler);

    // Set the priority of the interrupt
    IntPrioritySet(INT_GPIOE, 255);

    // Enable the interrupt in the NVIC
    IntEnable(INT_GPIOE);
		IntMasterEnable();
}










