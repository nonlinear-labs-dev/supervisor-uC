/******************************************************************************/
/** @file		coos.c
	@date		2016-06-01
    @author		[2013-07-07 DTZ]
*******************************************************************************/
#include "coos.h"
#include "pin_manipulation.h"
#include "pin_mapping.h"

#define COOS_MAX_TASKS			16												// max number of task the COOS should handle (memory size)

typedef struct
{
	void (* pTask)(void);														// pointer to the task
	int32_t countDown;															// delay (ticks) until the function will (next) be run
	int32_t period;																// interval (ticks) between subsequent run
	int32_t run;																	// incremented by the scheduler when task is due to execute
} sTask;

sTask COOS_taskArray[COOS_MAX_TASKS];											// array for the tasks

volatile int32_t taskOverflow = 0;												// signals a task overflow => turn on warning led
volatile int32_t checkTaskOverflow = 0;
volatile uint8_t sleep = 1;
volatile uint32_t systick_counter = 0;


/******************************************************************************/
/** @brief    	init everything with 0
*******************************************************************************/
void coos_init(void)
{
	uint8_t index;

	for (index = 0; index < COOS_MAX_TASKS; index++)
	{
		coos_delete_task(index);
	}
}



/******************************************************************************/
/** @brief		Function to add tasks to the task list
				- periodic tasks
				- one time tasks
    @param[in]  taskName - name of the task (function) that should be registered
	@param[in]	phase - one time delay in sysTicks to generate a phase (offset)
	@param[in]	period - interval in sysTicks between repeated execusions of
				the task
				0: execute only once
    @return		taskId - position in the taskArray
				-1: error
*******************************************************************************/
int32_t coos_add_task(void (* taskName)(), uint32_t phase, uint32_t period)
{
	uint8_t index = 0;

	while ((COOS_taskArray[index].pTask != 0) && (index < COOS_MAX_TASKS))		// check for space in the task array
	{
		index++;
	}

	if (index == COOS_MAX_TASKS)												// is the end of the task list accomplished?
	{
		return -1;																// task list is full: return error
	}

	/* there is a space in the taskArray - add task */
	COOS_taskArray[index].pTask	 	= taskName;
	COOS_taskArray[index].countDown = phase+1;
	COOS_taskArray[index].period 	= period;
	COOS_taskArray[index].run    	= 0;

	return index;																// so task can be deleted
}


 

/******************************************************************************/
/** @brief
    @param[in]	taskIndex
    			number of the task (id)
    @return		 0  everything ok
    			-1	error: no task at this location, nothing to delete
*******************************************************************************/
int32_t coos_delete_task(const uint8_t taskIndex)
{
	if (COOS_taskArray[taskIndex].pTask == 0)
	{
		return -1;																// error: no task at this location, nothing to delete
	}
	else
	{
		/* delete task */
		COOS_taskArray[taskIndex].pTask  	= 0x0000;
		COOS_taskArray[taskIndex].countDown = 0;
		COOS_taskArray[taskIndex].period 	= 0;
		COOS_taskArray[taskIndex].run 	 	= 0;
		return 0;																// everything ok
	}
}



void coos_delete_all_tasks(void)
{
	
}


uint32_t coos_get_systick_counter(void)
{
	return systick_counter;
}


void COOS_Start(void)
{
	// enable interrupts - start systick
}



void goto_sleep(void)
{
	// enter processor idle mode
}



void goto_artificial_sleep(void)
{
	while(sleep == 1)
	{
		asm volatile ("nop");
	};
	sleep = 1;
}

void 	coos_clear_pending(void)
{
		for (uint8_t index = 0; index < COOS_MAX_TASKS; index++)
		{
			COOS_taskArray[index].run = 0;
		}
}



/******************************************************************************/
/** @brief    	The dispatcher will run the registered tasks
    @param[]
    @return
*******************************************************************************/
void coos_dispatch(void)
{
	//DBG_Pod(POD_2, ON);															// monitor the duration of the dispatch function

	uint8_t index;

	for (index = 0; index < COOS_MAX_TASKS; index++)							// run the next task (if one is ready)
	{
		if (COOS_taskArray[index].run > 0)
		{
			(*COOS_taskArray[index].pTask)();									// run the task
			COOS_taskArray[index].run--;										// decrease the run flag, so postponed tasks will also be handled

			if (COOS_taskArray[index].period == 0)								// if one shot task: remove from taskArray
			{
				coos_delete_task(index);
			}
		}
	}

	//DBG_Pod(POD_2, OFF);														// monitor the duration of the dispatch function

	if (taskOverflow == 0)														// no task overflow -> everything all right -> goto sleep
	{
		sleep = 1;
		checkTaskOverflow--;
		// goto_artificial_sleep();
	}
	else
	{
		taskOverflow--; 														// task overflow -> try to catch up -> go an other round
	}
}



/******************************************************************************/
/** @brief    	Calculates when a task is due to run and sets the run flag when
 	 	 	 	it is. It will not execute any taks!!!
	@note		This function must be called every sysTick
*******************************************************************************/
void coos_update(void)
{
	uint8_t index;
	sleep = 0;
	systick_counter++;

#if 1 // check for task overrun
	if (checkTaskOverflow > 0)
	{
		taskOverflow++;															// error: Dispatch() took longer than one time slot
		//DBG_Led_Warning_On();
		//sys_taskoverflow_led_on();
		//COOS_Task_Add(DBG_Led_Warning_Off, 20000, 0);							// stays on for 2.5 sec
		//coos_add_task(sys_taskoverflow_led_off, 2500, 0);
	}
	else
	{
		checkTaskOverflow++;													// this flag must be reseted by Dispatch() before Dispatch is called again, otherwise: task overflow
	}
#endif

	for (index = 0; index < COOS_MAX_TASKS; index++)							// calculations are made in sysTicks
	{
		if (COOS_taskArray[index].pTask)										// check for registered task
		{
			COOS_taskArray[index].countDown--;

			if (COOS_taskArray[index].countDown <= 0)							// check if task is due to run / < 0 for one shot tasks
			{
				COOS_taskArray[index].run++;									// yes, task is due to run -> increase run-flag
				if (COOS_taskArray[index].period >= 1)
				{																// schedule periodic task to run again
					COOS_taskArray[index].countDown = COOS_taskArray[index].period;
				}
			}
		}
	}
}
