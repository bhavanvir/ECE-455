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

#define UNIT_TIME 5

xQueueHandle xQueue_DD_List_Request = 0;
xQueueHandle xQueue_DD_List_Response = 0;


xQueueHandle xQueue_To_Add = 0;
xQueueHandle xQueue_To_Remove = 0;

TaskHandle_t xHandlerTask1 = NULL;
TaskHandle_t xHandlerTask2 = NULL;
TaskHandle_t xHandlerTask3 = NULL;

TaskHandle_t xHandlers[3];

enum task_type { PERIODIC = 0, APERIODIC = 1 };
enum remove_flag { REMOVE = 1, KEEP = 0 };

typedef struct 
{
	enum task_type type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	uint32_t execution_time;
} dd_task;

typedef struct 
{
	dd_task *task;
	uint32_t task_id;
	struct dd_task_list *next_task;
} dd_task_list;

typedef struct 
{
	uint32_t execution_time;
	uint32_t period;
	uint32_t task_id;
	uint32_t type;
} dd_task_parameters;

typedef struct
{
	uint32_t task_id;
	uint32_t completion_time;
} dd_task_end_callback;

typedef struct {
	int active_count;
	int overdue_count;
	int compelete_count;
} dd_task_lists_count;

// Stubs for the functions
static void prvSetupHardware( void );
void create_dd_task( enum task_type type, uint32_t task_id, uint32_t absolute_deadline, uint32_t execution_time );
dd_task_list** get_active_dd_task_list( void );
dd_task_list** get_complete_dd_task_list( void );
dd_task_list** get_overdue_dd_task_list( void );
void Deadline_Driven_Scheduler( void *pvParameters );
void Deadline_Driven_Task_Generator1( void *pvParameters );
void Deadline_Driven_Task_Generator2( void *pvParameters );
void Deadline_Driven_Task_Generator3( void *pvParameters );
void Monitor_Task( void *pvParameters );
void vTemplateTaskCallback( TimerHandle_t xTimer );
void Template_Task( void *pvParameters );
void add_dd_task( dd_task_list** head, dd_task* task, uint32_t task_id );
dd_task* get_dd_task( dd_task_list** head, uint32_t task_id );
void remove_dd_task( dd_task_list** head, uint32_t task_id, int remove_flag );
void print_dd_task_list( dd_task_list** head );
void sort_dd_task_list( dd_task_list** head );
int count_tasks( dd_task_list** head );
void update_priorities_dd_task( dd_task_list** head );

dd_task_list *active_head = NULL;
dd_task_list *completed_head = NULL;
dd_task_list *overdue_head = NULL;

int main( void )
{
	prvSetupHardware();

	dd_task_parameters *task_parameters = (dd_task_parameters *)pvPortMalloc( 3 * sizeof( dd_task_parameters ) );
	if (task_parameters == NULL)
	    // Handle memory allocation error
		printf("Issue allocating memory!\n");

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
	task_parameters[2].period = 750;
	task_parameters[2].task_id = 2;
	task_parameters[2].type = PERIODIC;

	xQueue_To_Add = xQueueCreate( 3, sizeof( dd_task ) );
	xQueue_To_Remove = xQueueCreate( 3, sizeof( dd_task_end_callback ) );

	xQueue_DD_List_Request = xQueueCreate( 1, sizeof( uint32_t ) );
	xQueue_DD_List_Response = xQueueCreate( 1, sizeof( dd_task_lists_count ) );

	xTaskCreate( Deadline_Driven_Task_Generator1, "Generator 1", configMINIMAL_STACK_SIZE, task_parameters, 3, NULL );
	xTaskCreate( Deadline_Driven_Task_Generator2, "Generator 2", configMINIMAL_STACK_SIZE, task_parameters, 3, NULL );
	xTaskCreate( Deadline_Driven_Task_Generator3, "Generator 3", configMINIMAL_STACK_SIZE, task_parameters, 3, NULL );
	xTaskCreate( Deadline_Driven_Scheduler, "Scheduler", configMINIMAL_STACK_SIZE, NULL, 2, NULL );
	xTaskCreate( Monitor_Task, "Monitor", configMINIMAL_STACK_SIZE, NULL, 2, NULL );

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	vPortFree( task_parameters );

	// Should not reach here
	for(;;);
}

/* 
 * This function creates a task with the given parameters. It adds the task to the list of tasks and sets its
 * task type, task id, absolute deadline, and execution time. 
 */
void create_dd_task( enum task_type type, uint32_t task_id, uint32_t absolute_deadline, uint32_t execution_time )
{
	dd_task *task = (dd_task *)pvPortMalloc( sizeof( dd_task ) );
	TickType_t current_time = xTaskGetTickCount();

	//task->t_handle = handle;
	task->type = type;
	task->task_id = task_id;
	task->release_time = current_time;
	task->absolute_deadline = current_time + absolute_deadline;
	task->completion_time = -1;
	task->execution_time = execution_time;

	xQueueSend( xQueue_To_Add, (void *)&task, (TickType_t) UNIT_TIME );
}


/*  
 * This function implements the deadline driven scheduler. It is called by the
 * scheduler task that has been created by the scheduler task creator function.
 * It is passed in the parameter pvParameters, which is a pointer to a structure
 * of type TaskParameters_t. 
 */
void Deadline_Driven_Scheduler( void *pvParameters )
{
	dd_task *current_task;
	dd_task_end_callback* task_to_remove_data;
	dd_task_lists_count *lists_counts = (dd_task_lists_count *)pvPortMalloc( sizeof( dd_task_lists_count ) );

	uint32_t DDS_list_request_flag = 0;

	for(;;)
	{
		// Create tasks
		while( xQueueReceive(xQueue_To_Add, &current_task, 0) == pdTRUE )
		{
			char task_name[20];
			printf( "Task %u is created\n", current_task->task_id+1 );
			sprintf( task_name, "Task %u\0", current_task->task_id+1 );
			if( xHandlers[current_task->task_id] == NULL )
			{
				xTaskCreate( Template_Task, task_name, configMINIMAL_STACK_SIZE, current_task, 1, &xHandlers[current_task->task_id] );
				add_dd_task( &active_head, current_task, current_task->task_id );
				sort_dd_task_list( &active_head );
				update_priorities_dd_task( &active_head );
				remove_dd_task( &completed_head, current_task->task_id, REMOVE );
				remove_dd_task( &overdue_head, current_task->task_id, REMOVE );
			}
		}

		// Delete tasks
		while( xQueueReceive( xQueue_To_Remove, &task_to_remove_data, 0 ) == pdTRUE )
		{
			if( xHandlers[task_to_remove_data->task_id] != NULL )
			{
				printf( "Task %u is deleted\n", current_task->task_id+1 );
				vTaskSuspend( xHandlers[task_to_remove_data->task_id] );
				vTaskDelete( xHandlers[task_to_remove_data->task_id] );
				xHandlers[task_to_remove_data->task_id] = NULL;

				dd_task *completed_task = get_dd_task( &active_head, task_to_remove_data->task_id );
				completed_task->completion_time = task_to_remove_data->completion_time;

				remove_dd_task( &active_head,  task_to_remove_data->task_id, KEEP );
				if( completed_task->completion_time > completed_task->absolute_deadline )
					add_dd_task( &overdue_head, completed_task, completed_task->task_id );
				else
					add_dd_task( &completed_head, completed_task, completed_task->task_id );

				vPortFree( task_to_remove_data );
			}
		}

		if(xQueueReceive( xQueue_DD_List_Request, &DDS_list_request_flag, (TickType_t) UNIT_TIME ) == pdPASS )
		{
			lists_counts->active_count = count_tasks( &active_head );
			lists_counts->compelete_count = count_tasks( &completed_head );
			lists_counts->overdue_count = count_tasks( &overdue_head );

			printf( "\n" );
			printf( "ACTIVE LIST:\n" );
			print_dd_task_list( &active_head );
			printf( "OVERDUE LIST:\n" );
			print_dd_task_list( &overdue_head );
			printf( "COMPLETED LIST:\n" );
			print_dd_task_list( &completed_head );
			printf( "\n" );
			xQueueOverwrite( xQueue_DD_List_Response, &lists_counts );
		}
		vTaskDelay( pdMS_TO_TICKS( 10*UNIT_TIME ) );
	}
}


/* This function is a task that generates a periodic task ("Task 1") at a rate determined 
 * by the deadline driven scheduler. The task is generated by the task generator and then
 * passed to the deadline driven scheduler.
 */
void Deadline_Driven_Task_Generator1( void *pvParameters )
{
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvParameters;

	for(;;)
	{
		create_dd_task(
					   task_parameters[0].type,
					   task_parameters[0].task_id,
					   task_parameters[0].period,
					   task_parameters[0].execution_time
					   );

		vTaskDelay( pdMS_TO_TICKS( task_parameters[0].period ) );
	}
}

/* This function is a task that generates a periodic task ("Task 2") at a rate determined
 * by the deadline driven scheduler. The task is generated by the task generator and then
 * passed to the deadline driven scheduler.
 */
void Deadline_Driven_Task_Generator2( void *pvParameters )
{
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvParameters;

	for(;;)
	{
		create_dd_task(
					   task_parameters[1].type,
					   task_parameters[1].task_id,
					   task_parameters[1].period,
					   task_parameters[1].execution_time
					   );
		vTaskDelay( pdMS_TO_TICKS( task_parameters[1].period ) );
	}
}

/* This function is a task that generates a periodic task ("Task 3") at a rate determined
 * by the deadline driven scheduler. The task is generated by the task generator and then
 * passed to the deadline driven scheduler.
 */
void Deadline_Driven_Task_Generator3( void *pvParameters )
{
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvParameters;

	for(;;) 
	{
		create_dd_task(
					   task_parameters[2].type,
					   task_parameters[2].task_id,
					   task_parameters[2].period,
					   task_parameters[2].execution_time
					   );
		vTaskDelay( pdMS_TO_TICKS( task_parameters[2].period ) );
	}
}

/*
 * This task is responsible for monitoring the system health and
 * reporting any errors that occur. It also reports the system
 * status, such as the current state and mode of the system.
 */
void Monitor_Task( void *pvParameters )
{
	uint32_t set_flag = 1;

	dd_task_lists_count *lists_counts;

	for(;;)
	{
		xQueueOverwrite( xQueue_DD_List_Request, &set_flag );

		if( xQueueReceive( xQueue_DD_List_Response, &lists_counts, ( TickType_t ) UNIT_TIME ) == pdPASS )
			printf( "Active Count: %d  Overdue Count: %d  Complete Count: %d\n", lists_counts->active_count, lists_counts->overdue_count, lists_counts->compelete_count );

		vTaskDelay( pdMS_TO_TICKS( 100*UNIT_TIME ) );
	}
}

/*
 * This code is a template for a task that uses a timer to periodically 
 * execute a callback function, based on a task's parameters
 */
void Template_Task( void *pvParameters )
{
	dd_task *task = (dd_task *)pvParameters;

	TimerHandle_t xDD_Task_Timer = xTimerCreate( "DD Task Timer", pdMS_TO_TICKS( task->execution_time ), pdFALSE, (void *)task, vTemplateTaskCallback );
	xTimerStart( xDD_Task_Timer, pdMS_TO_TICKS( task->execution_time ) );
	for(;;)
		vTaskDelay( pdMS_TO_TICKS( UNIT_TIME ) );
}

/*
 * This function adds a dd_task to the dd_task_list, which is a linked list of dd_tasks.
 * The dd_task_list is a list of all the dd_tasks that have been created and are ready to run.
 * This function is called when a new dd_task is created.
 */
void add_dd_task( dd_task_list** head, dd_task* task, uint32_t task_id ) 
{
    dd_task_list* new_dd_task = (dd_task_list *)pvPortMalloc( sizeof( dd_task_list ) );
    new_dd_task->task_id = task_id;
    new_dd_task->task = task;
    new_dd_task->next_task = *head;
    *head = new_dd_task;
}

/*
 * Returns a pointer to the dd_task with the matching task_id
 * If no dd_task with the task_id exists, returns NULL
 */
dd_task* get_dd_task( dd_task_list** head, uint32_t task_id )
{
    dd_task_list* current = *head;
    while ( current != NULL )
	{
        if ( current->task->task_id == task_id )
            return current->task;
        current = current->next_task;
    }
    return NULL;
}

/* This function removes a task from the list of tasks. The remove_flag
 * can be set to 1 if the task is to be deleted from memory. If the
 * remove_flag is set to 0, then the task is not deleted from memory. 
 */
void remove_dd_task( dd_task_list** head, uint32_t task_id, int remove_flag )
{
    dd_task_list *current = *head, *prev = NULL;
    while ( current != NULL ) 
	{
        if ( current->task->task_id == task_id )
		{
            if ( prev == NULL )
                *head = current->next_task;
            else 
                prev->next_task = current->next_task;
            if ( remove_flag == 1 ) 
            	vPortFree( current->task );

            vPortFree( current );
            return;
        }
        prev = current;
        current = current->next_task;
    }
}

/*
 * This function prints the contents of a dd_task_list linked list. The
 * list is printed in the order it is stored in memory.
 */
void print_dd_task_list( dd_task_list** head ) 
{
    dd_task_list* current = *head;
    while ( current != NULL ) 
	{
		printf("\tTask %u with deadline %u\n", current->task->task_id+1, current->task->absolute_deadline);
    	current = current->next_task;
    }
}

/* This function takes a double linked list of tasks and sorts it based on the 
 * deadline of the tasks. The tasks are sorted in ascending order based on the 
 * deadline.
 */
void sort_dd_task_list( dd_task_list** head ) 
{
    dd_task_list *current, *search, *min;
    dd_task *temp;
    if ( *head == NULL )
        return;

    for ( current = *head; current->next_task != NULL; current = current->next_task ) 
	{
        min = current;
        for ( search = current->next_task; search != NULL; search = search->next_task ) 
            if ( search->task->absolute_deadline < min->task->absolute_deadline )
                min = search;

        if ( min != current ) 
		{
            temp = current->task;
            current->task = min->task;
            min->task = temp;
        }
    }
}

/*
 * This function updates the priority of each task in the list.
 * It is used by the scheduler to update the priority of each task
 * in the list.
 */
void update_priorities_dd_task( dd_task_list** head ) 
{
    dd_task_list* current = *head;
    int priority = 2;

    while ( current != NULL ) 
	{
    	if( xHandlers[current->task->task_id] != NULL )
    		vTaskPrioritySet( xHandlers[current->task->task_id], priority );

    	priority--;
        current = current->next_task;
    }
}

/*
 * This function counts the number of tasks in a task list.
 * It takes a pointer to the head of the task list as a parameter.
 * It returns the number of tasks in the task list.
 */
int count_tasks( dd_task_list** head )
{
	dd_task_list *current = *head;
	int tasks[3] = {0, 0, 0};
	int unique_count = 0;

	while( current != NULL ) 
	{
		if( tasks[current->task->task_id] == 0 ) 
		{
			tasks[current->task->task_id]++;
			unique_count++;
		}
		current = current->next_task;
	}

	return unique_count;
}


/* This is a callback function that is called when the timer expires.  The timer
 * handle, and the number of times that the timer has expired are passed into
 * this function.
 */
void vTemplateTaskCallback( TimerHandle_t xTimer ) 
{
	TickType_t current_time = xTaskGetTickCount();

	void *pvTimerID = pvTimerGetTimerID( xTimer );
	dd_task *task = (dd_task *)pvTimerID;
	task->completion_time = current_time;

	dd_task_end_callback *queue_values = (dd_task_end_callback *)pvPortMalloc( sizeof( dd_task_end_callback ) ) ;
	queue_values->task_id= task->task_id;
	queue_values->completion_time = task->completion_time;

	xQueueSend( xQueue_To_Remove, (void *)&queue_values, (TickType_t) UNIT_TIME );
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

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
}
