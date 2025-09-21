/*
 * @Author: Nagisa 2964793117@qq.com
 * @Date: 2025-08-07 22:06:30
 * @LastEditors: Nagisa 2964793117@qq.com
 * @LastEditTime: 2025-08-09 00:24:46
 * @FilePath: \MDK-ARMd:\project\git\robotcup\Robocup\F407VGT6\mcu_bsp\servo\upper.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */

#ifndef _UPPER_H_
#define _UPPER_H_
/*
 * @file upper.h 该工程包括了上层机构的逻辑和接口函数实现
 */
#include "main.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stdbool.h"

typedef enum
{
    down_location = 0,
    up_location = 1,
} upper_location;
void upper_move_distance(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);
void upper_to_target(upper_location target_position);

#endif