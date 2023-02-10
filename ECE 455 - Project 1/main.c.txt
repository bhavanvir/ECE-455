
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


/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );

//void GPIO_Init(GPIO_TypeDef* GPIOC+, GPIO_InitTypeDef* GPIO_InitStruct);
static void GPIO_Init_Shift_Register(void);



/*-----------------------------------------------------------*/

int main(void)
{

	// enable clock
	// GPIO_init
	GPIO_Init_Shift_Register();
	// set/reset
	// GPIO SetBits
	// GPIO ResetBits

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();



	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	return 0;
}

// GPIO ----------------------------------
//Red_Light -> PC0 -> Port C, Pin 0
static void GPIO_Init_Shift_Register (void)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	/* Enable the GPIO_LED Clock */
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	/* Configure the GPIO_LED pin */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
}


/*-- ADC functions ---------------------------------------------------------*/
//ADC on PC3
/*
static void Init_ADC()
{
// Enable clock for ADC1
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

// Init GPIOB for ADC input
    GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3|+;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}
*/

/*-----------------------------------------------------------*/


