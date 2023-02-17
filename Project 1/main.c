
/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define amber  	0
#define green  	1
#define red  	2
#define blue  	3

#define amber_led	LED3
#define green_led	LED4
#define red_led		LED5
#define blue_led	LED6

uint32_t Shift_Register_Reset = GPIO_Pin_8;
uint32_t Shift_Register_Data = GPIO_Pin_6;
uint32_t Shift_Register_Clock = GPIO_Pin_7;


/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
static void Manager_Task( void *pvParameters );
static void Blue_LED_Controller_Task( void *pvParameters );
static uint16_t ADC_Start_Conversion(void);
xQueueHandle xQueue_handle = 0;


/*-----------------------------------------------------------*/


static void GPIO_Init_C(void);

static void ADC_init(void);


int main(void)
{
	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();


	/* Enable the GPIO Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Enable clock for ADC1 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);


	//GPIO_Init
	//GPIO_Init_Shift_Register();
	GPIO_Init_C();
	ADC_init();
	printf("%u",ADC_Start_Conversion());

	//Reset(Clear) -> output = 0000
	GPIO_ResetBits(GPIOC, Shift_Register_Reset);
	for (int i = 0; i<10; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Reset);

	//Insert L + Shift -> output = 0000
	GPIO_ResetBits(GPIOC, Shift_Register_Data);
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	for (int i = 0; i<5; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);

	//Insert H + Shift -> output = 1000
	GPIO_SetBits(GPIOC, Shift_Register_Data);
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	for (int i = 0; i<5; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);
	GPIO_ResetBits(GPIOC, Shift_Register_Data);

	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xQueue_handle = xQueueCreate( 	mainQUEUE_LENGTH,		/* The number of items the queue can hold. */
							sizeof( uint16_t ) );	/* The size of each item the queue holds. */

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xQueue_handle, "MainQueue" );

	xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	xTaskCreate( Blue_LED_Controller_Task, "Blue_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);


	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
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
static void ADC_init()
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
	converted_data = ADC_GetConversionValue(ADC1);
	return converted_data;
}


/*-----------------------------------------------------------*/

static void Manager_Task( void *pvParameters ){
	uint16_t tx_data = amber;


	while(1)
	{

		if(tx_data == amber)
			//
		if(tx_data == green)
			//
		if(tx_data == red)
			//
		if(tx_data == blue)
			//

		if( xQueueSend(xQueue_handle,&tx_data,1000))
		{
			printf("Manager: %u ON!\n", tx_data);
			if(++tx_data == 4)
				tx_data = 0;
			vTaskDelay(1000);
		}
		else
		{
			printf("Manager Failed!\n");
		}
	}
}

/*-----------------------------------------------------------*/

static void Blue_LED_Controller_Task( void *pvParameters )
{/*
	uint16_t rx_data;
	while(1)
	{
		if(xQueueReceive(xQueue_handle, &rx_data, 500))
		{
			if(rx_data == blue)
			{
				vTaskDelay(250);
				STM_EVAL_LEDOff(blue_led);
				printf("Blue Off.\n");
			}
			else
			{
				if( xQueueSend(xQueue_handle,&rx_data,1000))
					{
						printf("BlueTask GRP (%u).\n", rx_data); // Got wwrong Package
						vTaskDelay(500);
					}
			}
		}
	}*/

	//Reset(Clear) -> output = 0000
	GPIO_ResetBits(GPIOC, Shift_Register_Reset);
	for (int i = 0; i<10; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Reset);

	//Insert L + Shift -> output = 0000
	GPIO_ResetBits(GPIOC, Shift_Register_Data);
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	for (int i = 0; i<5; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);

	//Insert H + Shift -> output = 1000
	GPIO_SetBits(GPIOC, Shift_Register_Data);
	GPIO_ResetBits(GPIOC, Shift_Register_Clock);
	for (int i = 0; i<5; i++);
	GPIO_SetBits(GPIOC, Shift_Register_Clock);
	GPIO_ResetBits(GPIOC, Shift_Register_Data);

}



/*-----------------------------------------------------------*/

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
