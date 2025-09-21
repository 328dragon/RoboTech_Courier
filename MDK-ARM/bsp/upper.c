#include "upper.h"

#include "usart.h"
// 1980中间960前面
extern upper_location now_upper_loacation;
extern upper_location target_upper_loacation;

void upper_move_distance(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    uint8_t cmd[16] = {0};

    // 装载命令
    cmd[0] = addr;                 // 地址
    cmd[1] = 0xFD;                 // 功能码
    cmd[2] = dir;                  // 方向
    cmd[3] = (uint8_t)(vel >> 8);  // 速度(RPM)高8位字节
    cmd[4] = (uint8_t)(vel >> 0);  // 速度(RPM)低8位字节
    cmd[5] = acc;                  // 加速度，注意：0是直接启动
    cmd[6] = (uint8_t)(clk >> 24); // 脉冲数(bit24 - bit31)
    cmd[7] = (uint8_t)(clk >> 16); // 脉冲数(bit16 - bit23)
    cmd[8] = (uint8_t)(clk >> 8);  // 脉冲数(bit8  - bit15)
    cmd[9] = (uint8_t)(clk >> 0);  // 脉冲数(bit0  - bit7 )

    cmd[10] = raF;  // 相位/绝对标志，false为相对运动，true为绝对值运动
    cmd[11] = snF;  // 多机同步运动标志，false为不启用，true为启用
    cmd[12] = 0x6B; // 校验字节

    // 发送命令
    HAL_UART_Transmit(&huart3, (uint8_t *)cmd, 13, 1000);
    vTaskDelay(10);
}
static void upper_move_location(upper_location now_location, upper_location target_position)
{
    int down_pulse = 200;
    int up_pulse = 1800;
	
    switch (now_location)
    {
    case down_location:
    {
        if (target_position == up_location)
        {
					for(int i=0;i<100;i++)
					{
					   upper_move_distance(5, 1, 300, 0.02, up_pulse - down_pulse, 1, 0); // 上升到最高位置
					}
         
            now_upper_loacation = up_location;
        }
        break;
    }

    case up_location:
    {
        if (target_position == down_location)
        {
								for(int i=0;i<100;i++)
					{
					 upper_move_distance(5, 0, 300, 0.02, up_pulse - down_pulse, 1, 0); // 降落到最低位置
					}
           
            now_upper_loacation = down_location;
        }
        break;
    }

    default:
        break;
    }
}

void upper_to_target(upper_location target_position)
{
    upper_move_location(now_upper_loacation, target_position);
}