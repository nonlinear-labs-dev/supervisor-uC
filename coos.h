/******************************************************************************/
/** @file		nl_sys_coos.h
    @date		2014-12-10
    @version	0.03
    @author		Daniel Tzschentke [2013-04-08]
    @brief		COOS co-operativ scheduler
				- 7 byte of memory for each task is needed
	@example	void main(void)
				{
					COOS_Init();
					// add tasks
					COOS_Start();

					while (1)
					{
						COOS_Dispatch();
					}
				}
	@details	If a task takes longer than the sysTick-time this will not
				affect the systems stability. But the next task will be dalayed.
				So this results in task jitter.
	@note 		Task overlaps can be prevented by using the phase variable of
				COOS_Task_Add();
	@ingroup	nl_sys_modules
	@todo		Error functions should use callback functions
*******************************************************************************/
#ifndef NL_SYS_COOS_H_
#define NL_SYS_COOS_H_

#include "stdint.h"

void 	coos_init(void);

int32_t coos_add_task(void (* task_name)(), uint32_t offset, uint32_t period);
int32_t coos_delete_task(const uint8_t task_index);
void    coos_delete_all_tasks(void);
uint32_t coos_get_systick_counter(void);
void 	coos_clear_pending(void);

void 	coos_dispatch(void);
void	coos_update(void);

#endif
