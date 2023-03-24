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

xQueueHandle xQueue_DD_List_Request = 0;
xQueueHandle xQueue_DD_List_Response = 0;


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
	struct dd_task *next_task;
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

typedef struct {
	uint32_t task_id;
	uint32_t completion_time;
} dd_task_end_callback;

typedef struct {
	int active_count;
	int overdue_count;
	int compelete_count;
} dd_task_lists_count;


static void prvSetupHardware(void);

/***************************************************************************************************************/
//Functions signatures

void create_dd_task(enum task_type type, uint32_t task_id, uint32_t absolute_deadline, uint32_t execution_time);
void delete_dd_task(uint32_t task_id);

dd_task_list** get_active_dd_task_list(void);
dd_task_list** get_complete_dd_task_list(void);
dd_task_list** get_overdue_dd_task_list(void);

void Deadline_Driven_Scheduler(void *pvParameters);
void Deadline_Driven_Task_Generator1(void *pvParameters);
void Deadline_Driven_Task_Generator2(void *pvParameters);
void Deadline_Driven_Task_Generator3(void *pvParameters);
void Monitor_Task(void *pvParameters);


void vTemplateTaskCallback(TimerHandle_t xTimer);
void Template_Task(void *pvParameters);

void check_handle_tasks_to_add_queue(void);
void check_handle_tasks_to_remove_queue(void);

void add_dd_task(dd_task_list** head, dd_task* task);
void remove_dd_task(dd_task_list** head, uint32_t task_id);
dd_task* get_dd_task(dd_task_list* head, uint32_t task_id);
void print_dd_task_list(dd_task_list* head);
void sort_dd_task_list(dd_task_list** head);
int count_tasks(dd_task_list* head);
/***************************************************************************************************************/

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
	xQueue_To_Remove = xQueueCreate(3, sizeof(dd_task_end_callback));

	xQueue_DD_List_Request = xQueueCreate(1, sizeof(uint32_t));
	xQueue_DD_List_Response = xQueueCreate(1, sizeof(dd_task_lists_count));

	xTaskCreate(Deadline_Driven_Task_Generator1, "Generator 1", configMINIMAL_STACK_SIZE, task_parameters, 2, NULL);
	xTaskCreate(Deadline_Driven_Task_Generator2, "Generator 2", configMINIMAL_STACK_SIZE, task_parameters, 2, NULL);
	xTaskCreate(Deadline_Driven_Task_Generator3, "Generator 3", configMINIMAL_STACK_SIZE, task_parameters, 2, NULL);
	xTaskCreate(Deadline_Driven_Scheduler, "Scheduler", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate(Monitor_Task, "Monitor", configMINIMAL_STACK_SIZE, NULL, 2, NULL);


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
	xQueueSend(xQueue_To_Add, (void *)&task, (TickType_t) UNIT_TIME);
}


/*-----------------------------------------------------------*/
void Deadline_Driven_Scheduler(void *pvParameters){

	dd_task *current_task;// = (dd_task*)pvPortMalloc(sizeof(dd_task));
	dd_task_end_callback* task_to_remove_data;
	dd_task_lists_count *lists_counts = (dd_task_lists_count*)pvPortMalloc(sizeof(dd_task_lists_count));

	uint32_t DDS_list_request_flag = 0;

	for(;;){
		// Create tasks
		while(xQueueReceive(xQueue_To_Add, &current_task, 0) == pdTRUE){
			char task_name[20];
			printf("task %u is created\n\n",current_task->task_id);
			sprintf(task_name, "Task %u\0", current_task->task_id+1);
			if(xHandlers[current_task->task_id] == NULL){
				xTaskCreate(Template_Task, task_name, configMINIMAL_STACK_SIZE, current_task, 1, &xHandlers[current_task->task_id]);
				add_dd_task(&active_head, current_task);
				sort_dd_task_list(&active_head);
				remove_dd_task(&completed_head,  current_task->task_id);
				remove_dd_task(&overdue_head,  current_task->task_id);
			}
			//vPortFree(current_task);
		}

		// Delete tasks
		while(xQueueReceive(xQueue_To_Remove, &task_to_remove_data, 0) == pdTRUE){
			if(xHandlers[task_to_remove_data->task_id] != NULL ){
				printf("task %u is being deleted\n\n",current_task->task_id);
				vTaskSuspend(xHandlers[task_to_remove_data->task_id]);
				vTaskDelete(xHandlers[task_to_remove_data->task_id]);
				xHandlers[task_to_remove_data->task_id] = NULL;

				dd_task *completed_task = get_dd_task(active_head, task_to_remove_data->task_id);
				completed_task->completion_time = task_to_remove_data->completion_time;

				remove_dd_task(&active_head,  task_to_remove_data->task_id);
				if(completed_task->completion_time > completed_task->absolute_deadline){
					add_dd_task(&overdue_head, completed_task);
				}else{
					add_dd_task(&completed_head, completed_task);
				}

				vPortFree(task_to_remove_data);
			}
		}



		if(xQueueReceive(xQueue_DD_List_Request, &DDS_list_request_flag, ( TickType_t ) UNIT_TIME ) == pdPASS){
			lists_counts->active_count = count_tasks(active_head);
			lists_counts->compelete_count = count_tasks(completed_head);
			lists_counts->overdue_count = count_tasks(overdue_head);

			xQueueOverwrite(xQueue_DD_List_Response, &lists_counts);
		}

		vTaskDelay(pdMS_TO_TICKS(UNIT_TIME));
	}
}

void Deadline_Driven_Task_Generator1(void *pvParameters){
	dd_task_parameters *task_parameters = (dd_task_parameters*)pvParameters;

	for(;;){
		create_dd_task(
					   task_parameters[0].type,
					   task_parameters[0].task_id,
					   task_parameters[0].period,
					   task_parameters[0].execution_time
					   );
		vTaskDelay(pdMS_TO_TICKS(task_parameters[0].period));
	}
}

void Deadline_Driven_Task_Generator2(void *pvParameters){
	dd_task_parameters *task_parameters = (dd_task_parameters*)pvParameters;

	for(;;){
		create_dd_task(
					   task_parameters[1].type,
					   task_parameters[1].task_id,
					   task_parameters[1].period,
					   task_parameters[1].execution_time
					   );
		vTaskDelay(pdMS_TO_TICKS(task_parameters[1].period));
	}
}

void Deadline_Driven_Task_Generator3(void *pvParameters){
	dd_task_parameters *task_parameters = (dd_task_parameters*)pvParameters;

	for(;;){
		create_dd_task(
					   task_parameters[2].type,
					   task_parameters[2].task_id,
					   task_parameters[2].period,
					   task_parameters[2].execution_time
					   );
		vTaskDelay(pdMS_TO_TICKS(task_parameters[2].period));
	}
}

void Monitor_Task(void *pvParameters){

	uint32_t set_flag = 1;

	dd_task_lists_count *lists_counts; //= (dd_task_lists_count*)pvPortMalloc(sizeof(dd_task_lists_count));

	for(;;){
		xQueueOverwrite(xQueue_DD_List_Request, &set_flag);

		if(xQueueReceive(xQueue_DD_List_Response, &lists_counts, ( TickType_t ) UNIT_TIME ) == pdPASS){
			printf("Active Count: %d  Overdue Count: %d  Complete Count: %d\n",lists_counts->active_count, lists_counts->overdue_count, lists_counts->compelete_count);
		}

		vTaskDelay(pdMS_TO_TICKS(1000*UNIT_TIME));
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
        printf("Task %u with deadline %u with completion time %u \n", current->task.task_id, current->task.absolute_deadline, current->task.completion_time);
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
	task->completion_time = current_time;

	dd_task_end_callback *queue_values = (dd_task_end_callback*)pvPortMalloc(sizeof(dd_task_end_callback));
	queue_values->task_id= task->task_id;
	queue_values->completion_time = task->completion_time;

	xQueueSend(xQueue_To_Remove, (void *)&queue_values, (TickType_t) UNIT_TIME);
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
