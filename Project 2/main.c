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

// Define the unit time
#define UNIT_TIME 5

// Define the quque handles
xQueueHandle xQueue_DD_List_Request = 0;
xQueueHandle xQueue_DD_List_Response = 0;
xQueueHandle xQueue_To_Add = 0;
xQueueHandle xQueue_To_Remove = 0;

// Define the task handles
TaskHandle_t xHandlerTask1 = NULL;
TaskHandle_t xHandlerTask2 = NULL;
TaskHandle_t xHandlerTask3 = NULL;
TaskHandle_t xHandlers[3];

// Enumerations for the task types and remove flag
enum task_type { PERIODIC = 0, APERIODIC = 1 };
enum remove_flag { REMOVE = 1, KEEP = 0 };

// Structure for the tasks
typedef struct 
{
	enum task_type type;
	uint32_t task_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
	uint32_t execution_time;
} dd_task;

// Structure for the task lists
typedef struct 
{
	dd_task *task;
	uint32_t task_id;
	struct dd_task_list *next_task;
} dd_task_list;

// Structure for the task parameters
typedef struct 
{
	uint32_t execution_time;
	uint32_t period;
	uint32_t task_id;
	uint32_t type;
} dd_task_parameters;

// Structure for the task end callback
typedef struct
{
	uint32_t task_id;
	uint32_t completion_time;
} dd_task_end_callback;

// Structure for the task lists count
typedef struct 
{
	int active_count;
	int overdue_count;
	int compelete_count;
} dd_task_lists_count;

// Stubs for the functions
static void prvSetupHardware( void );
void create_dd_task( enum task_type type, uint32_t task_id, uint32_t absolute_deadline, uint32_t execution_time );
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

	// The following code allocates three task parameters structures: one for each of three tasks.
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvPortMalloc( 3 * sizeof( dd_task_parameters ) );
	if (task_parameters == NULL)
	    // Handle memory allocation error
		printf("Issue allocating memory!\n");

	// The following code allocates three task handles: one for each of three tasks.
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

	// Queue to add tasks to the scheduler
	xQueue_To_Add = xQueueCreate( 3, sizeof( dd_task ) );
	// Queue to remove tasks from the scheduler
	xQueue_To_Remove = xQueueCreate( 3, sizeof( dd_task_end_callback ) );
	// Queue to request the task list
	xQueue_DD_List_Request = xQueueCreate( 1, sizeof( uint32_t ) );
	// Queue to respond with the task list
	xQueue_DD_List_Response = xQueueCreate( 1, sizeof( dd_task_lists_count ) );

	// Create the tasks defined within this file.
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
	// Create the task
	dd_task *task = (dd_task *)pvPortMalloc( sizeof( dd_task ) );
	// Get the current time
	TickType_t current_time = xTaskGetTickCount();

	// Set the task parameters
	task->type = type;
	task->task_id = task_id;
	task->release_time = current_time;
	task->absolute_deadline = current_time + absolute_deadline;
	task->completion_time = -1;
	task->execution_time = execution_time;

	// Add the task to the list of tasks
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
	// Define the current task
	dd_task *current_task;
	// Define the task to remove
	dd_task_end_callback *task_to_remove_data;
	// Define the task count
	dd_task_lists_count *lists_counts = (dd_task_lists_count *)pvPortMalloc( sizeof( dd_task_lists_count ) );

	// Define the flag to request the task list
	uint32_t DDS_list_request_flag = 0;

	for(;;)
	{
		// Check if there is a task to remove
		while( xQueueReceive(xQueue_To_Add, &current_task, 0) == pdTRUE )
		{
			char task_name[20];
			printf( "Task %u is created\n", current_task->task_id+1 );
			// Create the task name string 
			sprintf( task_name, "Task %u\0", current_task->task_id+1 );
			// Check if the task handle is NULL
			if( xHandlers[current_task->task_id] == NULL )
			{
				// Create the task
				xTaskCreate( Template_Task, task_name, configMINIMAL_STACK_SIZE, current_task, 1, &xHandlers[current_task->task_id] );
				// Add the task to the active list
				add_dd_task( &active_head, current_task, current_task->task_id );
				// Sort the active list
				sort_dd_task_list( &active_head );
				// Update the priorities of the tasks in the active list
				update_priorities_dd_task( &active_head );
				// Remove the task from the completed list
				remove_dd_task( &completed_head, current_task->task_id, REMOVE );
				// Remove the task from the overdue list
				remove_dd_task( &overdue_head, current_task->task_id, REMOVE );
			}
		}

		// Check if there is a task to remove
		while( xQueueReceive( xQueue_To_Remove, &task_to_remove_data, 0 ) == pdTRUE )
		{
			// Check if the task handle is not NULL
			if( xHandlers[task_to_remove_data->task_id] != NULL )
			{
				printf( "Task %u is deleted\n", current_task->task_id+1 );
				// Suspend the task
				vTaskSuspend( xHandlers[task_to_remove_data->task_id] );
				// Delete the task
				vTaskDelete( xHandlers[task_to_remove_data->task_id] );
				// Set the task handle to NULL
				xHandlers[task_to_remove_data->task_id] = NULL;

				// Add the task to the completed list
				dd_task *completed_task = get_dd_task( &active_head, task_to_remove_data->task_id );
				completed_task->completion_time = task_to_remove_data->completion_time;

				// Remove the task from the active list
				remove_dd_task( &active_head,  task_to_remove_data->task_id, KEEP );
				// If the completion time is greater than the absolute deadline, add the task to the overdue list
				if( completed_task->completion_time > completed_task->absolute_deadline )
					// Add the task to the overdue list
					add_dd_task( &overdue_head, completed_task, completed_task->task_id );
				else
					// Add the task to the completed list
					add_dd_task( &completed_head, completed_task, completed_task->task_id );

				// Free the memory allocated for the task to remove data
				vPortFree( task_to_remove_data );
			}
		}

		// Check if there is a request to print the task lists
		if(xQueueReceive( xQueue_DD_List_Request, &DDS_list_request_flag, (TickType_t) UNIT_TIME ) == pdPASS )
		{
			// Count the number of tasks in each list
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

			// Send the task lists count to the task list response queue
			xQueueOverwrite( xQueue_DD_List_Response, &lists_counts );
		}
		vTaskDelay( pdMS_TO_TICKS( 10*UNIT_TIME ) );
	}
}

/* 
 * This function is a task that generates a periodic task ("Task 1") at a rate determined 
 * by the deadline driven scheduler. The task is generated by the task generator and then
 * passed to the deadline driven scheduler.
 */
void Deadline_Driven_Task_Generator1( void *pvParameters )
{
	// Get the task parameters
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvParameters;

	for(;;)
	{
		// Create the task by calling the create_dd_task function and passing the task parameters
		create_dd_task(
					   task_parameters[0].type,
					   task_parameters[0].task_id,
					   task_parameters[0].period,
					   task_parameters[0].execution_time
					   );

		// Delay the task generator task for the period of the task
		vTaskDelay( pdMS_TO_TICKS( task_parameters[0].period ) );
	}
}

/* 
 * This function is a task that generates a periodic task ("Task 2") at a rate determined
 * by the deadline driven scheduler. The task is generated by the task generator and then
 * passed to the deadline driven scheduler.
 */
void Deadline_Driven_Task_Generator2( void *pvParameters )
{
	// Get the task parameters
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvParameters;

	for(;;)
	{
		// Create the task by calling the create_dd_task function and passing the task parameters
		create_dd_task(
					   task_parameters[1].type,
					   task_parameters[1].task_id,
					   task_parameters[1].period,
					   task_parameters[1].execution_time
					   );

		// Delay the task generator task for the period of the task
		vTaskDelay( pdMS_TO_TICKS( task_parameters[1].period ) );
	}
}

/* This function is a task that generates a periodic task ("Task 3") at a rate determined
 * by the deadline driven scheduler. The task is generated by the task generator and then
 * passed to the deadline driven scheduler.
 */
void Deadline_Driven_Task_Generator3( void *pvParameters )
{
	// Get the task parameters
	dd_task_parameters *task_parameters = (dd_task_parameters *)pvParameters;

	for(;;) 
	{
		// Create the task by calling the create_dd_task function and passing the task parameters
		create_dd_task(
					   task_parameters[2].type,
					   task_parameters[2].task_id,
					   task_parameters[2].period,
					   task_parameters[2].execution_time
					   );

		// Delay the task generator task for the period of the task
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
	// Define a flag to indicate that a request to print the task lists has been received
	uint32_t set_flag = 1;
	dd_task_lists_count *lists_counts;

	for(;;)
	{
		// Overwrite the task list request queue with a flag to indicate that a request to print the task lists has been received
		xQueueOverwrite( xQueue_DD_List_Request, &set_flag );

		// Check if the task lists count has been received
		if( xQueueReceive( xQueue_DD_List_Response, &lists_counts, ( TickType_t ) UNIT_TIME ) == pdPASS )
			printf( "Active Count: %d  Overdue Count: %d  Complete Count: %d\n", lists_counts->active_count, lists_counts->overdue_count, lists_counts->compelete_count );

		// Delay the monitor task for a period of 100*UNIT_TIME ms
		vTaskDelay( pdMS_TO_TICKS( 100*UNIT_TIME ) );
	}
}

/*
 * This code is a template for a task that uses a timer to periodically 
 * execute a callback function, based on a task's parameters
 */
void Template_Task( void *pvParameters )
{
	// Get the task parameters
	dd_task *task = (dd_task *)pvParameters;

	// Create a timer to periodically execute the callback function
	TimerHandle_t xDD_Task_Timer = xTimerCreate( "DD Task Timer", pdMS_TO_TICKS( task->execution_time ), pdFALSE, (void *)task, vTemplateTaskCallback );
	// Start the timer
	xTimerStart( xDD_Task_Timer, pdMS_TO_TICKS( task->execution_time ) );
	for(;;)
	    // Delay the task for a period of UNIT_TIME ms
		vTaskDelay( pdMS_TO_TICKS( UNIT_TIME ) );
}

/*
 * This function adds a dd_task to the dd_task_list, which is a linked list of dd_tasks.
 * The dd_task_list is a list of all the dd_tasks that have been created and are ready to run.
 * This function is called when a new dd_task is created.
 */
void add_dd_task( dd_task_list** head, dd_task* task, uint32_t task_id ) 
{
	// Allocate memory for the new dd_task
    dd_task_list* new_dd_task = (dd_task_list *)pvPortMalloc( sizeof( dd_task_list ) );
	// Set the task id and task pointer
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
	// Get the head of the list
    dd_task_list* current = *head;
	// Loop through the list until the end is reached or the task is found
    while ( current != NULL )
	{
		// Check if the task id matches the task id of the current task
        if ( current->task->task_id == task_id )
			// Return the task pointer
            return current->task;
		// Move to the next task
        current = current->next_task;
    }
    return NULL;
}

/* 
 * This function removes a task from the list of tasks. The remove_flag
 * can be set to 1 if the task is to be deleted from memory. If the
 * remove_flag is set to 0, then the task is not deleted from memory. 
 */
void remove_dd_task( dd_task_list** head, uint32_t task_id, int remove_flag )
{
	// Get the head of the list and the previous task
    dd_task_list *current = *head, *prev = NULL;
	// Loop through the list until the end is reached or the task is found
    while ( current != NULL ) 
	{
		// Check if the task id matches the task id of the current task
        if ( current->task->task_id == task_id )
		{
			// Remove the task from the list
            if ( prev == NULL )
				// If the task is the head of the list, set the head to the next task
                *head = current->next_task;
            else 
				// If the task is not the head of the list, set the previous task's next task to the current task's next task
                prev->next_task = current->next_task;
			// Free the memory allocated for the task
            if ( remove_flag == 1 ) 
            	vPortFree( current->task );

			// Free the memory allocated for the task list node
            vPortFree( current );
            return;
        }
		// Move to the next task
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
	// Get the head of the list
    dd_task_list* current = *head;
	// Loop through the list until the end is reached
    while ( current != NULL ) 
	{
		// Print the task id and the task's absolute deadline
		printf("\tTask %u with deadline %u\n", current->task->task_id+1, current->task->absolute_deadline);
    	current = current->next_task;
    }
}

/* 
 * This function takes a double linked list of tasks and sorts it based on the 
 * deadline of the tasks. The tasks are sorted in ascending order based on the 
 * deadline.
 */
void sort_dd_task_list( dd_task_list** head ) 
{
	// Define the head of the list and the current and search tasks
    dd_task_list *current, *search, *min;
	// Define a temporary task pointer
    dd_task *temp;
	// Check if the list is empty
    if ( *head == NULL )
        return;

	// Loop through the list until the end is reached
    for ( current = *head; current->next_task != NULL; current = current->next_task ) 
	{
		// Set the minimum task to the current task
        min = current;
		// Loop through the list until the end is reached
        for ( search = current->next_task; search != NULL; search = search->next_task )
			// Check if the search task's deadline is less than the minimum task's deadline 
            if ( search->task->absolute_deadline < min->task->absolute_deadline )
				// Set the minimum task to the search task
                min = search;

		// Swap the tasks if the minimum task is not the current task
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
	// Get the head of the list
    dd_task_list* current = *head;
	// Set the priority to 2, and have the range be from 0 to 2
    int priority = 2;

	// Loop through the list until the end is reached
    while ( current != NULL ) 
	{
		// Set the priority of the task
    	if( xHandlers[current->task->task_id] != NULL )
			// Set the priority of the task
    		vTaskPrioritySet( xHandlers[current->task->task_id], priority );

		// Move to the next task and decrement the priority
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
	// Get the head of the list
	dd_task_list *current = *head;
	// Set the number of tasks to 0
	int tasks[3] = {0, 0, 0};
	// Set the number of unique tasks to 0
	int unique_count = 0;

	// Loop through the list until the end is reached
	while( current != NULL ) 
	{
		// Check if the task has been counted
		if( tasks[current->task->task_id] == 0 ) 
		{
			// Increment the number of unique tasks
			tasks[current->task->task_id]++;
			unique_count++;
		}
		// Move to the next task
		current = current->next_task;
	}

	return unique_count;
}

/* 
 * This is a callback function that is called when the timer expires.  The timer
 * handle, and the number of times that the timer has expired are passed into
 * this function.
 */
void vTemplateTaskCallback( TimerHandle_t xTimer ) 
{
	// Get the current time
	TickType_t current_time = xTaskGetTickCount();

	// Get the task id from the timer parameter
	void *pvTimerID = pvTimerGetTimerID( xTimer );
	// Get the task from the task id
	dd_task *task = (dd_task *)pvTimerID;
	// Set the task's completion time
	task->completion_time = current_time;

	// Create a queue value to send to the queue
	dd_task_end_callback *queue_values = (dd_task_end_callback *)pvPortMalloc( sizeof( dd_task_end_callback ) ) ;
	// Set the task id and the completion time
	queue_values->task_id= task->task_id;
	queue_values->completion_time = task->completion_time;

	// Send the queue value to the queue
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