#include "bsp_tim_control.h"
#include "SR04.h"
extern SR04_t SR04_front;
int delay_no_conflict(int *delay_temp_count, int delay_time_base_multiple)
{
  (*delay_temp_count)++;
  if (*delay_temp_count >= delay_time_base_multiple)
  {
    *delay_temp_count = 0;
    return 1;
  }
  else
    return 0;
}



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance==TIM10)
{
  SR04_Elapsed_callback(&SR04_front);
}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
if(htim->Instance==TIM1)
{
  SR04_Echo_IC_callback(&SR04_front);
}

}