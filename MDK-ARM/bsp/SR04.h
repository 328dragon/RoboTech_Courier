#ifndef __SR04_H
#define __SR04_H
//#include "main.h"
#include "tim.h"
#include "stdio.h"
#include "stm32f4xx_hal_tim.h"

//利用顺序状态，和定时器计算
typedef struct SR04_Struct
{
    GPIO_TypeDef *Ttig_Port;       // 触发端口
    uint16_t Trig_Pin;             // 触发引脚
    TIM_HandleTypeDef *Echo_htim;       // 定时器句柄
    uint32_t Echo_TIM_Channel;         // 定时器通道
    float distant;                 // 测量距离
    uint32_t measure_Buf[3]; // 存放定时器计数值的数组
    uint8_t measure_Cnt ;       // 状态标志位
    int delay_temp_count; // 延时计数
    uint32_t high_time;            // 超声波模块返回的高电平时间
}
SR04_t;

void SR04_Register(SR04_t *SR04, GPIO_TypeDef *Trig_Port, uint16_t Trig_Pin,
				   TIM_HandleTypeDef *Echo_htim, uint32_t Echo_TIM_Channel);
void SR04_Elapsed_callback(SR04_t *_SR04);
void SR04_Echo_IC_callback(SR04_t *_SR04);
void SR04_GetData(SR04_t *_SR04);

#endif
