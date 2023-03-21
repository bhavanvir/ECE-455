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

/*-----------------------------------------------------------*/
/* Task prototypes */
void vTaskPrintHello(void *pvParameters);
void vTaskPrintWorld(void *pvParameters);
void vTaskPrintWindows(void *pvParameters);

/* Task handles */
TaskHandle_t xHandleHello = NULL;
TaskHandle_t xHandleWorld = NULL;

TaskHandle_t xHandler1 = NULL;
TaskHandle_t xHandler2 = NULL;
TaskHandle_t xHandler3 = NULL;

int main(void)
{
	prvSetupHardware();

	TaskHandle_t handlers[3];

	handlers[0] = xHandler1;
	handlers[1] = xHandler2;
	handlers[2] = xHandler3;

	/* Create task to print "Hello" */
	xTaskCreate(vTaskPrintHello, "HelloTask", configMINIMAL_STACK_SIZE, &handlers, tskIDLE_PRIORITY+1, &xHandleHello);

	/* Create task to print "World" */
	xTaskCreate(vTaskPrintWorld, "WorldTask", configMINIMAL_STACK_SIZE, &handlers, tskIDLE_PRIORITY+1, &xHandleWorld);


	/* Start the tasks and timer running. */
	vTaskStartScheduler();


	// Should not reach here
	for(;;);
}

/*-----------------------------------------------------------*/
/* Task to print "Hello" */
void vTaskPrintHello(void *pvParameters) {
  TaskHandle_t *handlers = (TaskHandle_t*)pvParameters;

  /* Create task to print "World" */
  xTaskCreate(vTaskPrintWindows, "WindowsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &handlers[0]);

  while(1) {
    printf("Hello");
    if(handlers[0] != NULL){
    	xTaskCreate(vTaskPrintWindows, "WindowsTask", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY+1, &handlers[0]);
    }
    vTaskDelay(pdMS_TO_TICKS(5000)); /* Delay for 1 second */
  }
}

/* Task to print "World" */
void vTaskPrintWorld(void *pvParameters) {
  TaskHandle_t *handlers = (TaskHandle_t*)pvParameters;
  while(1) {
    printf("World\n");
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Delay for 1 second */
    if(handlers[0] != NULL){
    	vTaskSuspend(handlers[0]);
    	vTaskDelete(handlers[0]);
    	handlers[0] = NULL;
    }

  }
}

/* Task to print "Windows" */
void vTaskPrintWindows(void *pvParameters) {
  while(1) {
    printf(" Windows \n");
    vTaskDelay(pdMS_TO_TICKS(1000)); /* Delay for 1 second */
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
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
