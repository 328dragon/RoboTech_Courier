#ifndef BSP_TIM_CONTROL_H
#define BSP_TIM_CONTROL_H
#include "stdint.h"
#include "tim.h"

int delay_no_conflict(int *delay_temp_count, int delay_time_base_multiple);

#endif // !BSP_TIM_CONTROL_H
