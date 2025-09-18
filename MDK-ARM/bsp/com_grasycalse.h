/*
 * @Author: Elaina
 * @Date: 2024-08-17 21:13:13
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-08-23 21:14:39
 * @FilePath: \MDK-ARM\Hardware\grayscale.h
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#ifndef __COM_GRAYSCALE_H
#define __COM_GRAYSCALE_H
#include "main.h"
#include "pid_template.h"
#include "controller.h"
struct GrascaleInterfaceConfig_t
{
    GPIO_TypeDef *grascale_port[7];
    uint16_t grascale_pin[7];
};
enum Com_GrayscaleMode_t
{
    GrasyCross,
    GrasyOnLine,
    OutLine,
};
class Com_Grayscale_t
{
public:
		Com_Grayscale_t()=default;
    
    Com_Grayscale_t(GrascaleInterfaceConfig_t config)
    {
       
        for (int i = 0; i < 4; i++)
        {
            grascale_port[i] = config.grascale_port[i];
            grascale_pin[i] = config.grascale_pin[i];
        }
        pid.target_update(0);
    }
    void read_data(void)
    {
        for (int i = 0; i < 4; i++)
        {
            data[i] = HAL_GPIO_ReadPin(grascale_port[i], grascale_pin[i]);
        }

    }
    float ReturnXControl(void)
    {
        read_data();
        float error = 0;

        error -= data[2]*1;
        error += data[3]*1;

        control = -pidx.update(error);
        return control;
    }
    float ReturnCotorl(void)
    {
        read_data();
        float error = 0;

        error -= data[2]*1;
        error += data[3]*1;

        control = -pid.update(error);
        return control;
    }
    bool IsCurrentMode(Com_GrayscaleMode_t mode)
    {
        read_data();
        switch (mode)
        {
        case GrasyCross:
            return data[1] && data[2] && data[3];
            break;
        case GrasyOnLine:
            return data[2] || data[3];
            break;
        case OutLine:
            return data[1] && data[2] && data[3];
            break;
        default:
            return false;
            break;
        }
    }

protected:
    float control;
    uint8_t data[4];
    GPIO_TypeDef *grascale_port[4];
    uint16_t grascale_pin[4];
    pid_base_template_t<float,float> pid = pid_base_template_t<float,float>({0.1, 0, 0.1, -3, 3});
    pid_base_template_t<float,float> pidx = pid_base_template_t<float,float>({0.025, 0, 0.01, -0.3, 0.3});
};

#endif
