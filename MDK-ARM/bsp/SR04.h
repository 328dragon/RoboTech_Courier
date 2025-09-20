#ifndef __SR04_H
#define __SR04_H
#include "main.h"
#include "tim.h"
#include "stdio.h"
 
#define TRIG_H    HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,1)
#define TRIG_L   HAL_GPIO_WritePin(GPIOE,GPIO_PIN_9,0)
 
void delay_us(uint32_t us);
void SR04_GetData(void);
 
#endif