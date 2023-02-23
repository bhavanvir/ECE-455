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

/*-----------------------------------------------------------*/
#define MAX_POT 4095

uint32_t Green_Light = GPIO_Pin_2;
uint32_t Yellow_Light = GPIO_Pin_1;
uint32_t Red_Light = GPIO_Pin_0;
uint32_t Shift_Register_Reset = GPIO_Pin_8;
uint32_t Shift_Register_Data = GPIO_Pin_6;
uint32_t Shift_Register_Clock = GPIO_Pin_7;

xQueueHandle xQueue_POT = 0;
xQueueHandle xQueue_Traffic_Generated = 0;
xQueueHandle xQueue_Lights_Status = 0;
TimerHandle_t  xGreen_Light = 0;
TimerHandle_t  xYellow_Light = 0;
TimerHandle_t  xRed_Light = 0;

static void prvSetupHardware( void );
static void Traffic_Flow_Adjustment_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );
static void vGreenLightCallBack( TimerHandle_t xTimer );
static void vYellowLightCallBack( TimerHandle_t xTimer );
static void vRedLightCallBack( TimerHandle_t xTimer );
static uint16_t ADC_Start_Conversion(void);
static void GPIO_Init_C(void);
static void ADC_Init_C(void);
/*-----------------------------------------------------------*/

void set_low(uint32_t Shift_Register_Data, uint32_t Shift_Register_Clock) {
	GPIO_ResetBits(GPIOC, Shift_Register_Data);
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);
}

void set_high(uint32_t Shift_Register_Data, uint32_t Shift_Register_Clock) {
	GPIO_SetBits(GPIOC, Shift_Register_Data);
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);
	GPIO_ResetBits(GPIOC, Shift_Register_Data);
}

void led_reset(uint32_t Shift_Register_Reset) {
	GPIO_ResetBits(GPIOC, Shift_Register_Reset);
	GPIO_SetBits(GPIOC, Shift_Register_Reset);
}

void array_to_led(int car_pattern[19]) {
	for(int i = 18; i >= 0; i--){
		if(car_pattern[i] == 1)
			set_high(Shift_Register_Data, Shift_Register_Clock);
		else
			set_low(Shift_Register_Data, Shift_Register_Clock);
	}
}

int main(void)
{
	prvSetupHardware();

	/* Enable the GPIO Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable clock for ADC1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_Init_C();
	ADC_Init_C();

	xQueue_POT = xQueueCreate(1, sizeof(uint16_t));
	xQueue_Traffic_Generated = xQueueCreate(1, sizeof(uint16_t));
	xQueue_Lights_Status = xQueueCreate(1, sizeof(uint32_t));

	/*
	led_reset(Shift_Register_Reset);
	int pattern[] = {0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
	array_to_led(pattern);
	led_reset(Shift_Register_Reset);
	*/

	xTaskCreate(Traffic_Flow_Adjustment_Task, "Traffic_Flow_Adjustment_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Traffic_Generator_Task, "Traffic_Generator_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xTaskCreate(Traffic_Light_State_Task, "Traffic_Light_State_Task", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
	xGreen_Light = xTimerCreate("Green_Light_Timer", pdMS_TO_TICKS(2000), pdFALSE, (void *) 0, vGreenLightCallBack);
	xYellow_Light = xTimerCreate("Yellow_Light_Timer", pdMS_TO_TICKS(1000), pdFALSE, (void *) 0, vYellowLightCallBack);
	xRed_Light = xTimerCreate("Red_Light_Timer", pdMS_TO_TICKS(2000), pdFALSE, (void *) 0, vRedLightCallBack);
	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	/* Should not reach here. */
	for(;;);
}



static void GPIO_Init_C (void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;

	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 ;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}

//ADC on PC3
static void ADC_Init_C()
{
    GPIO_InitTypeDef GPIO_InitStruct;

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Init ADC1
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;
	ADC_InitStruct.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
	ADC_InitStruct.ADC_NbrOfConversion = 1;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ScanConvMode = DISABLE;
	ADC_Init(ADC1, &ADC_InitStruct);
	ADC_Cmd(ADC1, ENABLE);

	// Select input channel for ADC1
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_84Cycles);
}

static uint16_t ADC_Start_Conversion()
{
	uint16_t converted_data;
	// Start ADC conversion
	ADC_SoftwareStartConv(ADC1);
	// Wait until conversion is finish
	while (!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC));

	// Get the value
	converted_data =  ADC_GetConversionValue(ADC1);
	return converted_data;
}


/*-----------------------------------------------------------*/
void Traffic_Flow_Adjustment_Task( void *pvParameters ){
	int POT = 0;

	while(1){
		POT = ADC_Start_Conversion();
		if(xQueueOverwrite(xQueue_POT, &POT) == pdPASS){
			printf("Traffic_Flow_Adjustment_Task: %d\n", POT);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

void Traffic_Generator_Task( void *pvParameters ){
	int POT = 0;

	while(1){
		if(xQueuePeek(xQueue_POT, &POT, (TickType_t) 1000) == pdPASS){
			printf("Traffic_Generator_Task: %d\n", POT);
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}


void Traffic_Light_State_Task( void *pvParameters ){
	xTimerStart(xGreen_Light, 0);
	xQueueOverwrite(xQueue_Lights_Status, &Green_Light);

	/*
	while(1){
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	*/
}

void System_Display_Task( void *pvParameters ){

}

void vGreenLightCallBack( TimerHandle_t xTimer ){
	xTimerStart(xYellow_Light, 0);
	xQueueOverwrite(xQueue_Lights_Status, &Yellow_Light);
}

void vYellowLightCallBack( TimerHandle_t xTimer ){
	int POT = 0;

	if(xQueuePeek(xQueue_POT, &POT, (TickType_t) 1000) == pdPASS){
		xTimerChangePeriod(xRed_Light, ceil(3 - (POT/MAX_POT)*2), 0);
		xQueueOverwrite(xQueue_Lights_Status, &Red_Light);
	}
}

void vRedLightCallBack( TimerHandle_t xTimer ){
	int POT = 0;

	if(xQueuePeek(xQueue_POT, &POT, (TickType_t) 1000) == pdPASS){
		xTimerChangePeriod(xGreen_Light, ceil(3 + (POT/MAX_POT)*2), 0);
		xQueueOverwrite(xQueue_Lights_Status, &Green_Light);
	}
}

/*-----------------------------------------------------------*/

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

/*-----------------------------------------------------------*/

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

/*-----------------------------------------------------------*/

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

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
