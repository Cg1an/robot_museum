#include "main.h"

void Start_INS_Task(void const * argument)
{
	for(;;)
	{
	 INS_task();
		if(offset_OK_flag==0)offset_OK_flag = cali_gyro_hook();

	osDelay(1);
	}
}

