/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
#include <math.h>
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"

// Define the maximum value of the potentiometer and the unit time
#define MAX_POT 4095
#define UNIT_TIME 1000

// Define the GPIO pins for the LEDs and the Shift Registers
uint32_t Green_Light = GPIO_Pin_2;
uint32_t Yellow_Light = GPIO_Pin_1;
uint32_t Red_Light = GPIO_Pin_0;
uint32_t Shift_Register_Reset = GPIO_Pin_8;
uint32_t Shift_Register_Data = GPIO_Pin_6;
uint32_t Shift_Register_Clock = GPIO_Pin_7;

// Define the queues
xQueueHandle xQueue_POT = 0;
xQueueHandle xQueue_Traffic_Generated = 0;
xQueueHandle xQueue_Lights_Status = 0;
// Define the timers
TimerHandle_t  xGreen_Light = 0;
TimerHandle_t  xYellow_Light = 0;
TimerHandle_t  xRed_Light = 0;

// Stubs for the functions
static void prvSetupHardware( void );
static void Traffic_Flow_Adjustment_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );
static void vGreenLightCallBack( TimerHandle_t xTimer );
static void vYellowLightCallBack( TimerHandle_t xTimer );
static void vRedLightCallBack( TimerHandle_t xTimer );
static uint16_t ADC_Start_Conversion(void);
static void GPIO_Init_C( void );
static void ADC_Init_C( void );

/*
 * Given the data and clock pins for the shift register, this function will set the corresponding LED to low.
 */
void set_low( uint32_t Shift_Register_Data, uint32_t Shift_Register_Clock ) 
{
	GPIO_ResetBits( GPIOC, Shift_Register_Data );
	GPIO_ResetBits( GPIOC, Shift_Register_Clock );
	GPIO_SetBits( GPIOC, Shift_Register_Clock );
}

/*
 * Given the data and clock pins for the shift register, this function will set the corresponding LED to high.
 */
void set_high( uint32_t Shift_Register_Data, uint32_t Shift_Register_Clock ) 
{
	GPIO_SetBits( GPIOC, Shift_Register_Data );
	GPIO_ResetBits( GPIOC, Shift_Register_Clock );
	GPIO_SetBits( GPIOC, Shift_Register_Clock );
	GPIO_ResetBits( GPIOC, Shift_Register_Data );
}

/*
 * Given an array representing the traffic generated, this function will display the traffic generated on the LEDs,
 * with the first element of the array being the leftmost LED and the last element of the array being the rightmost LED. 
 */
void array_to_led( int generated_cars[19] ) 
{
	// For each element of the array, if the element is 1, set the corresponding LED to high, otherwise set the corresponding LED to low
	for( int i = 18; i >= 0; i-- )
	{
		if( generated_cars[i] == 1 )
			// Set the corresponding LED to high
			set_high( Shift_Register_Data, Shift_Register_Clock );
		else
			// Set the corresponding LED to low
			set_low( Shift_Register_Data, Shift_Register_Clock );
	}
}

int main( void )
{
	prvSetupHardware();

	// Enable the GPIO Clock
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOC, ENABLE );
	// Enable clock for ADC1
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_ADC1, ENABLE );

	// Initialize the GPIO pins for the LEDs and the Shift Registers
	GPIO_Init_C();
	// Initialize the ADC
	ADC_Init_C();

	// Create the queues, one for the potentiometer, one for the traffic generated, and one for the lights status, each with a size of 1
	xQueue_POT = xQueueCreate( 1, sizeof(uint16_t) );
	xQueue_Traffic_Generated = xQueueCreate( 1, sizeof(uint16_t) );
	xQueue_Lights_Status = xQueueCreate( 1, sizeof(uint32_t) );

	// Create the tasks, one for the traffic flow adjustment, one for the traffic generator, one for the traffic light state, and one for the system display, each with a priority of 1
	xTaskCreate( Traffic_Flow_Adjustment_Task, "Traffic_Flow_Adjustment_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( Traffic_Generator_Task, "Traffic_Generator_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( Traffic_Light_State_Task, "Traffic_Light_State_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	xTaskCreate( System_Display_Task, "System_Display_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL );
	// Create the timers, one for the green light, one for the yellow light, and one for the red light, each with a period of 2 seconds (equivalent to 2 UNIT_TIME)
	xGreen_Light = xTimerCreate( "Green_Light_Timer", pdMS_TO_TICKS(2*UNIT_TIME), pdFALSE, (void *) 0, vGreenLightCallBack );
	xYellow_Light = xTimerCreate( "Yellow_Light_Timer", pdMS_TO_TICKS(UNIT_TIME), pdFALSE, (void *) 0, vYellowLightCallBack );
	xRed_Light = xTimerCreate( "Red_Light_Timer", pdMS_TO_TICKS(2*UNIT_TIME), pdFALSE, (void *) 0, vRedLightCallBack );
	// Start the tasks and timer running.
	vTaskStartScheduler();

	// Should not reach here.
	for( ;; );
}

/*
 * Adapted from the provided lab slides.
 */
static void GPIO_Init_C ( void )
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	// Configure the GPIO_LED pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOC, &GPIO_InitStructure );
}

/*
 * Adapted from the provided lab slides.
 */
static void ADC_Init_C()
{
    GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init( GPIOC, &GPIO_InitStruct );

	// Init ADC1
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init( ADC1, &ADC_InitStruct );
	ADC_Cmd( ADC1, ENABLE );

	// Select input channel for ADC1
	ADC_RegularChannelConfig( ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles );
}

/*
 * Adapted from the provided lab slides.
 */
static uint16_t ADC_Start_Conversion()
{
	uint16_t converted_data;
	// Start ADC conversion
	ADC_SoftwareStartConv( ADC1 );
	// Wait until conversion is finish
	while (!ADC_GetFlagStatus( ADC1, ADC_FLAG_EOC ) );

	// Get the value
	converted_data =  ADC_GetConversionValue( ADC1 );
	return converted_data;
}

/*
 * Adapted from the provided lab slides.
 */
void shift_about_stop( int generated_cars[20] )
{
	// Index 8 represents the stop line, where indices 0-7 represent possible locations for cars to be in
	for( int i = 8; i > 0; i-- )
	{
		// If we currently have no car in the designated index, we continue shifting until the current LED is on
		if( generated_cars[i] == 0 )
		{
			// Set the current index to the value of the previous index
			generated_cars[i] = generated_cars[i-1];
			// Set the previous index to 0
			generated_cars[i-1] = 0;
		}
	}

	// Index 9 represents the first index after the stop line, and we continue shifting after 9 up through 19
	for( int i = 19; i > 9; i-- )
	{
		// Set the current index to the value of the previous index
		generated_cars[i] = generated_cars[i-1];
		// Set the previous index to 0
		generated_cars[i-1] = 0;
	}
}

/*
 * The task runs periodically, at period of one second, to check the value of potentiometer and send it to the queue.
 */
void Traffic_Flow_Adjustment_Task( void *pvParameters )
{
	// Initialize the value of potentiometer to 0
	int POT = 0;
	while( 1 )
	{
		// Read the value of potentiometer and store it in POT
		POT = ADC_Start_Conversion();
		// For debugging purposes, we print the value of potentiometer
		printf( "POT: %d \n", POT );
		// Send the value of potentiometer to the queue
		if( xQueueOverwrite( xQueue_POT, &POT ) == pdPASS )
			// Delay the task for one second (equivalent to one UNIT_TIME)
			vTaskDelay( pdMS_TO_TICKS( UNIT_TIME ) );
	}
}

/*
 * The task set the traffic_generated_queue to 1 to signal the creation of new car at a specific period.
 * The period of generation changes according to the value of potentiometer as read from the queue.
 */
void Traffic_Generator_Task( void *pvParameters )
{
	// Initialize the value of potentiometer to 0, and the flag to 1
	int POT = 0;
	int generate_flag = 1;

	// The period equation was built with a factor of 12 which was reached through trial and error by monitoring LED traffic spread
	while( 1 )
	{
		// If the queue is not empty, we read the value of potentiometer and calculate the period of generation
		if( xQueuePeek( xQueue_POT, &POT, (TickType_t) UNIT_TIME) == pdPASS )
		{
			xQueueOverwrite( xQueue_Traffic_Generated, &generate_flag );
			// Delay the task for the calculated period
			vTaskDelay( pdMS_TO_TICKS( (int)ceil( 12*((double)(MAX_POT-POT)/MAX_POT) )*UNIT_TIME) );
		}
	}
}

/*
 * Initiates a series of callbacks to update the traffic lights in the right order and the right periods according to the potentiometer values.
 * We start by starting the first traffic light in our chain, the green light.
 */
void Traffic_Light_State_Task( void *pvParameters )
{
	// Start the green light timer, which will trigger the callback to turn on the yellow light upon expiration
	xTimerStart( xGreen_Light, 0 );
	// Overwrite the queue with the green light status
	xQueueOverwrite( xQueue_Lights_Status, &Green_Light );

	while( 1 ) vTaskDelay( pdMS_TO_TICKS( UNIT_TIME ) );
}

void System_Display_Task( void *pvParameters )
{
	uint32_t current_light;
	uint16_t generate_flag;
	// Initialize a 20 wide array representing all car LEDs
	int generated_cars[20] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

	// Reset the Shift Register for the current run
	GPIO_SetBits( GPIOC, Shift_Register_Reset );
	while( 1 )
	{
		// Get the current traffic light to be turned on
		if( xQueuePeek( xQueue_Lights_Status, &current_light, (TickType_t) UNIT_TIME ) == pdPASS )
		{
			// Turn off all traffic lights
			GPIO_ResetBits( GPIOC, Green_Light );
			GPIO_ResetBits( GPIOC, Yellow_Light );
			GPIO_ResetBits( GPIOC, Red_Light );
			// Turn the current traffic light on
			GPIO_SetBits( GPIOC, current_light );
		}

		// Given the generated_cars array, turn on each LED represented by a 1 and vice versa
		array_to_led( generated_cars );

		// If the current light is green, continue shifting cars forward
		if( current_light == Green_Light )
			for( int i = 19; i > 0; i-- ) generated_cars[i] = generated_cars[i-1];
		else
			// If the current light is yellow or red, shift cars behind and after the stop line forwards
			shift_about_stop( generated_cars );

		// Always set the first LED to off, to anticipate the next run
		generated_cars[0] = 0;
		// Get the generate flag value, and set first LED accordingly
		if( (xQueueReceive( xQueue_Traffic_Generated, &generate_flag, (TickType_t) UNIT_TIME) == pdPASS ) && ( generate_flag == 1 ) )
			generated_cars[0] = 1;

		// Delay the task for one second (equivalent to one UNIT_TIME)
		vTaskDelay( pdMS_TO_TICKS( UNIT_TIME ) );
	}
}

/*
 * We continue running through our chain, by running the yellow light after we have called our green light.
 * We start the yellow light and have it running for a constant period of UNIT_TIME, which represents 1 second.
 */
void vGreenLightCallBack( TimerHandle_t xTimer )
{
	xTimerStart( xYellow_Light, UNIT_TIME );
	// For debugging purposes, we print the period of the yellow light
	printf( "xYellow_Light: %d\n", UNIT_TIME );
	xQueueOverwrite( xQueue_Lights_Status, &Yellow_Light );
}

/*
 * With low a potentiometer value read from the xQueue_POT queue, the period for red increases, with a maximum period of 8
 * seconds and minimum of 1 second. This function sets the flag to the next light in the chain, in this case being red.
 */
void vYellowLightCallBack( TimerHandle_t xTimer )
{
	// Initialize the value of potentiometer to 0
	int POT = 0;

	// If the queue is not empty, we read the value of potentiometer and calculate the period of red light
	if(xQueuePeek( xQueue_POT, &POT, (TickType_t) UNIT_TIME) == pdPASS )
	{
		// Start the red light timer, which will trigger the callback to turn on the green light upon expiration, and set the period 
		xTimerChangePeriod( xRed_Light, pdMS_TO_TICKS( (int)ceil( 8*((double)(MAX_POT-POT)/MAX_POT) )*UNIT_TIME ), 0 );
		// For debugging purposes, we print the period of the red light
		printf( "xRed_Light: %d\n", (int)ceil( 8*((double)(MAX_POT-POT)/MAX_POT) )*UNIT_TIME );
		// Overwrite the queue with the red light status
		xQueueOverwrite( xQueue_Lights_Status, &Red_Light );
	}
}

/*
 * With low a potentiometer value read from the xQueue_POT queue, the period for green increases, with a maximum period of 8
 * seconds and minimum of 1 second. This function sets the flag to the next light in the chain, in this case being green, to
 * restart the cycle once again.
 */
void vRedLightCallBack( TimerHandle_t xTimer )
{
	int POT = 0;

	if( xQueuePeek( xQueue_POT, &POT, (TickType_t) UNIT_TIME ) == pdPASS )
	{
		// Start the green light timer, which will trigger the callback to turn on the yellow light upon expiration, and set the period
		xTimerChangePeriod( xGreen_Light, pdMS_TO_TICKS( (int)ceil( (double)POT/MAX_POT*8 )*UNIT_TIME ), 0 );
		// For debugging purposes, we print the period of the green light
		printf( "xGreen_Light: %d\n", (int)ceil( (double)POT/MAX_POT*8 )*UNIT_TIME );
		// Overwrite the queue with the green light status
		xQueueOverwrite( xQueue_Lights_Status, &Green_Light );
	}
}

/*
 * Adapted from the provided lab slides.
 */
void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}

/*
 * Adapted from the provided lab slides.
 */
void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );
}
