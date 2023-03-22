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

TaskHandle_t xHandlerTask1 = NULL;
TaskHandle_t xHandlerTask2 = NULL;
TaskHandle_t xHandlerTask3 = NULL;

TaskHandle_t xHandlers[3];

enum task_type {PERIODIC = 0, APERIODIC = 1};

typedef struct {
	enum task_type type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	uint32_t execution_time;
} dd_task;

typedef struct {
	dd_task task;
	struct dd_task_list *next_task;
} dd_task_list;

typedef struct {
	uint32_t execution_time;
	uint32_t period;
	uint32_t task_id;
	uint32_t type;
} dd_task_parameters;

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware(void);
void create_dd_task(enum task_type type, uint32_t task_id, uint32_t absolute_deadline, uint32_t execution_time);
void delete_dd_task(uint32_t task_id);
dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_complete_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);
void Deadline_Driven_Scheduler(void *pvParameters);
void Deadline_Driven_Task_Generator(void *pvParameters);
void check_handle_tasks_to_add_queue(void);
void check_handle_tasks_to_remove_queue(void);
void vTemplateTaskCallback(TimerHandle_t xTimer);
void Template_Task(void *pvParameters);
void add_dd_task(dd_task_list** head, dd_task* task);
void remove_dd_task(dd_task_list** head, uint32_t task_id);
dd_task* get_dd_task(dd_task_list* head, uint32_t task_id);
void print_dd_task_list(dd_task_list* head);
void sort_dd_task_list(dd_task_list** head);
int count_tasks(dd_task_list* head);

/*-----------------------------------------------------------*/
// Put back in DDS after debugging
dd_task_list *active_head = NULL;
dd_task_list *completed_head = NULL;
dd_task_list *overdue_head = NULL;

int main(void)
{
	prvSetupHardware();

	dd_task_parameters *task_parameters = (dd_task_parameters*)pvPortMalloc(3 * sizeof(dd_task_parameters));
	if (task_parameters == NULL) {
	    // Handle memory allocation error
		printf("Issue allocating memory!\n");
	}

	xHandlers[0] = xHandlerTask1;
	xHandlers[1] = xHandlerTask2;
	xHandlers[2] = xHandlerTask3;

	// Task 1
	task_parameters[0].execution_time = 95;
	task_parameters[0].period = 500;
	task_parameters[0].task_id = 0;
	task_parameters[0].type = PERIODIC;
	// Task 2
	task_parameters[1].execution_time = 150;
	task_parameters[1].period = 500;
	task_parameters[1].task_id = 1;
	task_parameters[1].type = PERIODIC;
	// Task 3
	task_parameters[2].execution_time = 250;
	task_parameters[2].period = 500;
	task_parameters[2].task_id = 2;
	task_parameters[2].type = PERIODIC;

	xQueue_To_Add = xQueueCreate(3, sizeof(dd_task));
	xQueue_To_Remove = xQueueCreate(3, sizeof(uint32_t));

	xTaskCreate(Deadline_Driven_Task_Generator, "Generator", configMINIMAL_STACK_SIZE, task_parameters, 2, NULL);
	xTaskCreate(Deadline_Driven_Scheduler, "Scheduler", configMINIMAL_STACK_SIZE, NULL, 3, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	vPortFree(task_parameters);

	// Should not reach here
	for(;;);
}

/*-----------------------------------------------------------*/
void create_dd_task(enum task_type type, uint32_t task_id, uint32_t absolute_deadline, uint32_t execution_time){
	dd_task *task = (dd_task*)pvPortMalloc(sizeof(dd_task));
	TickType_t current_time = xTaskGetTickCount();

	//task->t_handle = handle;
	task->type = type;
	task->task_id = task_id;
	task->release_time = current_time;
	task->absolute_deadline = current_time + absolute_deadline;
	task->completion_time = -1;
	task->execution_time = execution_time;
	xQueueSend(xQueue_To_Add, &task, (TickType_t) UNIT_TIME);
}
/*
int get_active_dd_task_list(void){
	dd_task_list** active_dd_list;

	//write_to_queue(task_id, &xQueue_Active_DD_List_Request);

	//while(read_from_queue(xQueue_Active_DD_List_Response, &active_dd_list) != pdPASS);

	return active_dd_list;
}

int get_complete_dd_task_list(void){
	//same as previous function
}

int get_overdue_dd_task_list(void){
	//same as previous function
}
*/
/*-----------------------------------------------------------*/

// Helper functions

/*
check_handle_get_request_queue(active_dd_task_list, complete_dd_task_list, overdue_dd_task_list){
	uint32_t active_flag = 0;
	uint32_t completed_flag = 0;
	uint32_t overdue_flag = 0;

	if(xQueueReceive(xQueue_Active_DD_List_Request, &active_flag, ( TickType_t ) UNIT_TIME ) == pdPASS)
		xQueueOverwrite(xQueue_Active_DD_List_Response, count_dd_list(active_dd_task_list));

	if(xQueueReceive(xQueue_Completed_DD_List_Request, &completed_flag, ( TickType_t ) UNIT_TIME ) == pdPASS)
		xQueueOverwrite(xQueue_Completed_DD_List_Response, count_dd_list(complete_dd_task_list));

	if(xQueueReceive(xQueue_Overdue_DD_List_Request, &overdue_flag, ( TickType_t ) UNIT_TIME ) == pdPASS)
		xQueueOverwrite(xQueue_Overdue_DD_List_Response, count_dd_list(overdue_dd_task_list));
}
*/


/*-----------------------------------------------------------*/
void Deadline_Driven_Scheduler(void *pvParameters){
	dd_task *current_task = (dd_task*)pvPortMalloc(sizeof(dd_task));
	uint32_t task_id_to_remove;

	for(;;){
		// Create tasks
		while(xQueueReceive(xQueue_To_Add, &current_task, 0) == pdTRUE){
			char task_name[20];
			sprintf(task_name, "Task %u", current_task->task_id+1);
			if(xHandlers[current_task->task_id] == NULL){
				xTaskCreate(Template_Task, task_name, configMINIMAL_STACK_SIZE, current_task, 1, &xHandlers[current_task->task_id]);
				add_dd_task(&active_head, current_task);
				sort_dd_task_list(&active_head);
			}
		}

		// Delete tasks
		while(xQueueReceive(xQueue_To_Remove, &task_id_to_remove, 0) == pdTRUE){
			if(xHandlers[task_id_to_remove] != NULL ){
					vTaskSuspend(xHandlers[task_id_to_remove]);
					vTaskDelete(xHandlers[task_id_to_remove]);
					xHandlers[task_id_to_remove] = NULL;

					dd_task *completed_task = get_dd_task(active_head, task_id_to_remove);
					remove_dd_task(&active_head, task_id_to_remove);
					add_dd_task(&completed_head, completed_task);
			}
		}

		vTaskDelay(pdMS_TO_TICKS(UNIT_TIME));
	}
}

void Deadline_Driven_Task_Generator(void *pvParameters){
	dd_task_parameters *task_parameters = (dd_task_parameters*)pvParameters;
	int i = 0;

	for(;;){
		create_dd_task(
					   task_parameters[i%3].type,
					   task_parameters[i%3].task_id,
					   task_parameters[i%3].period,
					   task_parameters[i%3].execution_time
					   );
		vTaskDelay(pdMS_TO_TICKS(task_parameters[i%3].period));
		i++;

		print_dd_task_list(completed_head);
		int count = count_tasks(completed_head);
		printf("%d\n\n", count);
	}
}

void Template_Task(void *pvParameters){
	dd_task *task = (dd_task *)pvParameters;

	TimerHandle_t xDD_Task_Timer = xTimerCreate("DD Task Timer", pdMS_TO_TICKS(task->execution_time), pdFALSE, (void *)task, vTemplateTaskCallback);
	xTimerStart(xDD_Task_Timer, pdMS_TO_TICKS(task->execution_time));
	for(;;){
		vTaskDelay(pdMS_TO_TICKS(UNIT_TIME));
	}
}

/*-----------------------------------------------------------*/
void add_dd_task(dd_task_list** head, dd_task* task) {
    dd_task_list* new_dd_task = (dd_task_list*)pvPortMalloc(sizeof(dd_task_list));
    new_dd_task->task = *task;
    new_dd_task->next_task = *head;
    *head = new_dd_task;
}

void remove_dd_task(dd_task_list** head, uint32_t task_id) {
    dd_task_list *current = *head, *prev = NULL;
    while (current != NULL) {
        if (current->task.task_id == task_id) {
            if (prev == NULL) {
                // If the node to be removed is the head of the list
                *head = current->next_task;
            } else {
                // If the node to be removed is not the head of the list
                prev->next_task = current->next_task;
            }
            vPortFree(current);
            return;
        }
        prev = current;
        current = current->next_task;
    }
}

dd_task* get_dd_task(dd_task_list* head, uint32_t task_id) {
    dd_task_list* current = head;
    while (current != NULL) {
        if (current->task.task_id == task_id) {
            return &(current->task);
        }
        current = current->next_task;
    }
    return NULL;
}

void print_dd_task_list(dd_task_list* head) {
    dd_task_list* current = head;
    while (current != NULL) {
        printf("Task %u with deadline %u\n", current->task.task_id, current->task.absolute_deadline);
        current = current->next_task;
    }
}

void sort_dd_task_list(dd_task_list** head) {
    dd_task_list *current, *search, *min;
    dd_task temp;
    if (*head == NULL) {
        return;
    }
    for (current = *head; current->next_task != NULL; current = current->next_task) {
        min = current;
        for (search = current->next_task; search != NULL; search = search->next_task) {
            if (search->task.absolute_deadline < min->task.absolute_deadline) {
                min = search;
            }
        }
        if (min != current) {
            temp = current->task;
            current->task = min->task;
            min->task = temp;
        }
    }
}

int count_tasks(dd_task_list* head){
	dd_task_list *current = head;

	int tasks[3] = {0, 0, 0};
	int unique_count = 0;
	while(current != NULL) {
		if(tasks[current->task.task_id] == 0) {
			tasks[current->task.task_id]++;
			unique_count++;
		}
		current = current->next_task;
	}

	return unique_count;
}
/*-----------------------------------------------------------*/

void vTemplateTaskCallback(TimerHandle_t xTimer){
	TickType_t current_time = xTaskGetTickCount();
	void *pvTimerID = pvTimerGetTimerID(xTimer);
	dd_task *task = (dd_task *)pvTimerID;
	task->completion_time = current_time - task->release_time;

	xQueueSend(xQueue_To_Remove, &task->task_id, (TickType_t) UNIT_TIME);
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
