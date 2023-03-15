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
#define UNIT_TIME 5

xQueueHandle xQueue_Active_DD_List_Response = 0;
xQueueHandle xQueue_Active_DD_List_Request = 0;

xQueueHandle xQueue_Completed_DD_List_Response = 0;
xQueueHandle xQueue_Completed_DD_List_Request = 0;

xQueueHandle xQueue_Overdue_DD_List_Response = 0;
xQueueHandle xQueue_Overdue_DD_List_Request = 0;

xQueueHandle xQueue_To_Add = 0;
xQueueHandle xQueue_To_Remove = 0;

xQueueHandle xQueue_Universal_Clock = 0;

TimerHandle_t xUniversal_Clock = 0;

enum task_type {PERIODIC = 0, APERIODIC = 1};

typedef struct {
	TaskHandle_t t_handle;
	enum task_type type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
} dd_task;

typedef struct {
	dd_task task;
	struct dd_task_list *next_task;
} dd_task_list;

typedef struct {
	int execution_time;
	int period;
	int task_id;
	int type;
} dd_task_parameters;

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );
void create_dd_task(TaskHandle_t t_handle, enum task_type type, uint32_t task_id, uint32_t absolute_deadline);
void delete_dd_task(uint32_t task_id);
dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_complete_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);
void Deadline_Driven_Scheduler(void *pvParameters);
void Deadline_Driven_Task_Generator(void *pvParameters);

/*-----------------------------------------------------------*/

int main(void)
{
	prvSetupHardware();

	dd_task_parameters *task_parameters = (dd_task_parameters*)pvPortMalloc(3 * sizeof(dd_task_parameters));
	if (task_parameters == NULL) {
	    // Handle memory allocation error
	}

	// Task 1
	task_parameters[0].execution_time = 95;
	task_parameters[0].period = 500;
	task_parameters[0].task_id = 1;
	task_parameters[0].type = PERIODIC;
	// Task 2
	task_parameters[1].execution_time = 150;
	task_parameters[1].period = 500;
	task_parameters[1].task_id = 2;
	task_parameters[1].type = PERIODIC;
	// Task 3
	task_parameters[2].execution_time = 250;
	task_parameters[2].period = 500;
	task_parameters[2].task_id = 3;
	task_parameters[2].type = PERIODIC;

	xQueueHandle xQueue_To_Add = xQueueCreate(3, sizeof(dd_task));
	xQueueHandle xQueue_Universal_Clock = 0;

	xTaskCreate(Deadline_Driven_Task_Generator, "Deadline-Driven Task Generator", configMINIMAL_STACK_SIZE, task_parameters, 1, NULL);

	//xUniversal_Clock = xTimerCreate("Universal Clock", pdMS_TO_TICKS(), pdTRUE, (void *) 0, vUniversalTimerCallback);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	vPortFree(task_parameters);

	// Should not reach here
	for(;;);
}

/*-----------------------------------------------------------*/

void create_dd_task(TaskHandle_t handle, enum task_type type, uint32_t task_id, uint32_t absolute_deadline){
	dd_task *task = (dd_task*)pvPortMalloc(sizeof(dd_task));
	task->t_handle = handle;
	task->type = type;
	task->task_id = task_id;
	task->release_time = 0;
	task->absolute_deadline = absolute_deadline;
	task->completion_time = 0;

}

void delete_dd_task(uint32_t task_id){

}

dd_task_list** get_active_dd_task_list(void){

}

dd_task_list** get_complete_dd_task_list(void){

}

dd_task_list** get_overdue_dd_task_list(void){

}

/*-----------------------------------------------------------*/

void Deadline_Driven_Scheduler(void *pvParameters){

}

void Deadline_Driven_Task_Generator(void *pvParameters){
	dd_task_parameters *task_parameters = (dd_task_parameters*)pvParameters;

	int i = 0;
	for(;;){
		TaskHandle_t *t_handle;
		create_dd_task(t_handle, task_parameters[i%3].type, task_parameters[i%3].task_id, task_parameters[i%3].period);
		i++;
	}
}

void Template_Task(void *pvParameters){

}

void vUniversalTimerCallback(TimerHandle_t xTimer){

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

