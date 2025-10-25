/*
 * @Author: Elaina
 * @Date: 2024-08-17 21:13:10
 * @LastEditors: chaffer-cold 1463967532@qq.com
 * @LastEditTime: 2024-10-18 23:09:14
 * @FilePath: \MDK-ARMg:\project\stm32\f405rgb6\08_guosai\Core\Src\maincpp.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
// 2（0）   1（1）
//
// 3（2）   4（3）

#include "maincpp.h"
#define PI 3.1415926535
#include "FreeRTOS.h"
#include "Kinematic.h"
#include "bsp_usart.h"
#include "controller.h"
#include "host_control.hpp"
#include "planner.h"
#include "stepmotorZDT.hpp"
#include "task.h"
#include "ch040.h"
#include "gw_grasycalse.h"
#include "com_grasycalse.h"
#include "logic.h"
#define code_mode 0
#include "SR04.h"
#include "upper.h"
#define HC08_length 9 //几个按键
float tar_rad = 0;
float last_rad = 0;
int turn_ok = 1;
int straight_ok = 1;
int staright_dir = 0;
int Calibration_flag=0;
int put_flag=0;
int get_flag=0;
// 实例化Map并将初始点设置成startInfo
StepMotorZDT_t *stepmotor_list_ptr[4];
TaskHandle_t Chassic_control_handle; // 底盘更新
TaskHandle_t main_cpp_handle;        // 主函数
TaskHandle_t Planner_update_handle;  // 轨迹规划
TaskHandle_t gray_read_handle;       // 灰度传感器
TaskHandle_t upper_move_handle;      // 上层机构
USARTInstance StepMotorUart;         // 步进电机串口实例
USARTInstance ch040Uart;             // ch040串口实例
USARTInstance HC08Uart;              // HC08
// TaskHandle_t Ontest_handle;
// void ontest(void *pvParameters);
GW_grasycalse::Gw_Grayscale_t Gw_GrayscaleSensor_front;

SR04_t SR04_front;
int sr04_ok = 0;

// 上升电机
upper_location now_upper_loacation = up_location;
upper_location target_upper_loacation = up_location;
int upper_flag = 0;
Dir dragon_forbid = DOWN;

void OnChassicControl(void *pvParameters);
void OnKinematicUpdate(void *pvParameters);
void Onmaincpp(void *pvParameters);
void OnPlannerUpdate(void *pvParameters);
void StepCallBack(void *param);
void ch040CallBack(void *param);
void HC08CallBack(void *param);
void gray_read_task(void *pvParameters);
void upper_move_task(void *pvParameters);
// 现在有三种控制方法
/*一是基于自身坐标系下的速度闭环*/
/*二是基于大地坐标系下的速度闭环*/
/*三是基于自身坐标系下的位置闭环*/
/*一只要一开始给一个控制量*/
/*二与三需要实时更新*/
void main_cpp(void)
{

  // 感为灰度
  Gw_GrayscaleSensor_front = GW_grasycalse::Gw_Grayscale_t(&hi2c3, GW_GRAY_ADDR_DEF);

  // 超声波
  HAL_TIM_Base_Start_IT(&htim10);
  SR04_Register(&SR04_front, GPIOE, GPIO_PIN_9, &htim1, TIM_CHANNEL_4);
  // 串口配置
  USART_Init_Config_s init_config;
  // 底盘控制串口
  init_config.param = nullptr;
  init_config.recv_buff_size = 50;
  init_config.usart_handle = &huart3;
  init_config.module_callback = StepCallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&StepMotorUart, &init_config);
  // ch040陀螺仪串口
  init_config.usart_handle = &huart6;
  init_config.recv_buff_size = 100;
  init_config.module_callback = ch040CallBack; // 这里传入的是静态函数,需要注意参数类型
  USARTRegister(&ch040Uart, &init_config);
  // HC08遥控器
  init_config.usart_handle = &huart1;
  init_config.recv_buff_size = 20;
  init_config.module_callback = HC08CallBack;
  USARTRegister(&HC08Uart, &init_config);
  // 电机实例化
  stepmotor_list_ptr[0] = new StepMotorZDT_t(2, &huart3, false, 0);
  stepmotor_list_ptr[1] = new StepMotorZDT_t(1, &huart3, false, 1);
  stepmotor_list_ptr[2] = new StepMotorZDT_t(3, &huart3, false, 0);
  stepmotor_list_ptr[3] = new StepMotorZDT_t(4, &huart3, true, 1);
  KinematicOdom = KinematicOdom_t(0.2535);
  // 需要用reinterpret_cast转换到父类指针类型
  Controller = StepController_t(stepmotor_list_ptr);
  // 上位机控制
  HostPtr =
      new HostControl_t(&huart4);
  // 任务开启
  BaseType_t ok2 = xTaskCreate(OnChassicControl, "Chassic_control", 600, NULL,
                               3, &Chassic_control_handle);
  BaseType_t ok3 =
      xTaskCreate(Onmaincpp, "main_cpp", 800, NULL, 4, &main_cpp_handle);
  BaseType_t ok4 = xTaskCreate(OnPlannerUpdate, "Planner_update", 1000, NULL, 4,
                               &Planner_update_handle);
  BaseType_t ok5 = xTaskCreate(gray_read_task, "gray_read_task", 300, NULL, 2, &gray_read_handle);
  BaseType_t ok6 = xTaskCreate(upper_move_task, "upper_move_distance", 100, NULL, 2, &upper_move_handle);
  //  BaseType_t ok10 = xTaskCreate(ontest, "ontest_work", 200, NULL, 2,
  //                            &Ontest_handle);
  //   if (ok != pdPASS || ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS)
  if (ok2 != pdPASS || ok3 != pdPASS || ok4 != pdPASS)
  {
    // 任务创建失败，进入死循环
    while (1)
    {
      // uart_printf("create task failed\n");
    }
  }
}

void upper_move_task(void *pvParameters)
{
  while (1)
  {
    sr04_ok = 0;
    upper_to_target(target_upper_loacation);
    SR04_GetData(&SR04_front);
    sr04_ok = 1;
    vTaskDelay(100);
  }
}

void gray_read_task(void *pvParameters)
{
  while (Gw_GrayscaleSensor_front.gw_ping())
  {
    vTaskDelay(100);
  }
  while (1)
  {
    Gw_GrayscaleSensor_front.read_data();
    // map_left_down._current_distance = SR04_front.distant;
    vTaskDelay(10);
  }
}

void HC08_Ctrl()
{

  // 只管转向
  if (turn_ok == 1)
  {
    turn_ok = 0;
    if (tar_rad != last_rad)
    {
			KinematicOdom.CurrentOdom.x = 0;
      KinematicOdom.CurrentOdom.y = 0;
      auto &turn_move = Planner.LoactaionCloseControl({0, 0, tar_rad - last_rad}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
      while (turn_move.isResolved() == false)
      {

        vTaskDelay(50);
      }
      last_rad = tar_rad;
      vTaskDelay(500);
      Controller.Clear();
      vTaskDelay(50);
    }
    turn_ok = 1;
  }
  if (straight_ok == 1)
  {
    straight_ok = 0;
    if (staright_dir == 1)
    {
			  KinematicOdom.CurrentOdom.x = 0;
      KinematicOdom.CurrentOdom.y = 0;
      auto &straight_move = Planner.LoactaionCloseControl({0.16, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
      while (straight_move.isResolved() == false)
      {
        vTaskDelay(50);
      }
      KinematicOdom.CurrentOdom.x = 0;
      KinematicOdom.CurrentOdom.y = 0;
    }
    else if (staright_dir == -1)
    {
			  KinematicOdom.CurrentOdom.x = 0;
      KinematicOdom.CurrentOdom.y = 0;
      auto &straight_move = Planner.LoactaionCloseControl({-0.16, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
      while (straight_move.isResolved() == false)
      {
        vTaskDelay(50);
      }
      KinematicOdom.CurrentOdom.x = 0;
      KinematicOdom.CurrentOdom.y = 0;
    }
    staright_dir = 0;
    straight_ok = 1;
  }
	if(Calibration_flag==1)
	{
		Calibration_flag=0;
		KinematicOdom.CurrentOdom.x=0;
		KinematicOdom.CurrentOdom.y=0;	

	 while (Gw_GrayscaleSensor_front.IsCurrentMode(GW_grasycalse::GrasyOnLine)==0)
  {
    // 沿线前进
    Controller.SetVelTarget({0.13, Gw_GrayscaleSensor_front.ReturnXControl(), 0});
    vTaskDelay(50);
  }
 Controller.SetVelTarget({0, 0, 0});
  Controller.Clear();
  vTaskDelay(50);
	auto &straight_move= Planner.LoactaionCloseControl({0.16,0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }		
		
	}
	
}
void Onmaincpp(void *pvParameters)
{
  UNUSED(pvParameters);
  vTaskDelay(1000);
  ch040.setYawZero();
#if code_mode == 1
  // 第一个轮回
  while (Gw_GrayscaleSensor_front.IsCurrentMode(GW_grasycalse::GrasyOnLine) == 0)
  {
    // 沿线前进
    Controller.SetVelTarget({0.13, Gw_GrayscaleSensor_front.ReturnXControl(), 0});
    vTaskDelay(50);
  }
  Controller.SetVelTarget({0, 0, 0});
  vTaskDelay(50);

  auto &straight_move = Planner.LoactaionCloseControl({0.26, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }
  KinematicOdom.CurrentOdom.x = 0;
  KinematicOdom.CurrentOdom.y = 0;
  vTaskDelay(50);
  // 带角度
  straight_move = Planner.LoactaionCloseControl({0, 0, PI / 2}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }
  // Controller.Clear();
  vTaskDelay(50);
  /// 下一个轮回
  while (Gw_GrayscaleSensor_front.IsCurrentMode(GW_grasycalse::GrasyOnLine) == 0)
  {
    // 沿线前进
    Controller.SetVelTarget({0.13, Gw_GrayscaleSensor_front.ReturnXControl(), 0});
    vTaskDelay(50);
  }
  Controller.SetVelTarget({0, 0, 0});
  Controller.Clear();
  vTaskDelay(50);

  straight_move = Planner.LoactaionCloseControl({0.16, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }

  KinematicOdom.CurrentOdom.x = 0;
  KinematicOdom.CurrentOdom.y = 0;
  vTaskDelay(50);
  // 带角度
  straight_move = Planner.LoactaionCloseControl({0, 0, -PI / 2}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }

  // Controller.Clear();
  vTaskDelay(50);

  // 第三个轮回
  while (Gw_GrayscaleSensor_front.IsCurrentMode(GW_grasycalse::GrasyOnLine) == 0)
  {
    // 沿线前进
    Controller.SetVelTarget({0.13, Gw_GrayscaleSensor_front.ReturnXControl(), 0});
    vTaskDelay(50);
  }
  Controller.SetVelTarget({0, 0, 0});
  Controller.Clear();
  vTaskDelay(50);
  straight_move = Planner.LoactaionCloseControl({0.16, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }

  KinematicOdom.CurrentOdom.x = 0;
  KinematicOdom.CurrentOdom.y = 0;
  vTaskDelay(50);
  // 带角度
  straight_move = Planner.LoactaionCloseControl({0, 0, -PI / 2}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }
  // Controller.Clear();
  vTaskDelay(50);
  // 第四个轮回
  while (Gw_GrayscaleSensor_front.IsCurrentMode(GW_grasycalse::GrasyOnLine) == 0)
  {
    // 沿线前进
    Controller.SetVelTarget({0.13, Gw_GrayscaleSensor_front.ReturnXControl(), 0});
    vTaskDelay(50);
  }
  Controller.SetVelTarget({0, 0, 0});
  Controller.Clear();
  vTaskDelay(50);
  straight_move = Planner.LoactaionCloseControl({0.16, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }

  KinematicOdom.CurrentOdom.x = 0;
  KinematicOdom.CurrentOdom.y = 0;
  vTaskDelay(50);
  // 带角度
  straight_move = Planner.LoactaionCloseControl({0, 0, -PI / 2}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }
  // Controller.Clear();
  vTaskDelay(50);
  // 第五个轮回
  while (Gw_GrayscaleSensor_front.IsCurrentMode(GW_grasycalse::GrasyOnLine) == 0)
  {
    // 沿线前进
    Controller.SetVelTarget({0.13, Gw_GrayscaleSensor_front.ReturnXControl(), 0});
    vTaskDelay(50);
  }
  Controller.SetVelTarget({0, 0, 0});
  Controller.Clear();
  vTaskDelay(50);
  straight_move = Planner.LoactaionCloseControl({0.16, 0, 0}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }

  KinematicOdom.CurrentOdom.x = 0;
  KinematicOdom.CurrentOdom.y = 0;
  vTaskDelay(50);
  // 带角度
  straight_move = Planner.LoactaionCloseControl({0, 0, PI}, 0.5, 1.0, {0.1, 0.1, 0.1}, false);
  while (straight_move.isResolved() == false)
  {
    vTaskDelay(50);
  }
  // Controller.Clear();
  vTaskDelay(50);
#endif
  while (1)
  {
#if code_mode == 0
    HC08_Ctrl();
#endif
    vTaskDelay(200);
  }
}

void OnPlannerUpdate(void *pvParameters)
{
  UNUSED(pvParameters);
  uint16_t last_tick = xTaskGetTickCount();

  while (1)
  {
    uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
    last_tick = xTaskGetTickCount();
    Planner.update(dt);
    vTaskDelay(40);
  }
}
void OnChassicControl(void *pvParameters)
{
  UNUSED(pvParameters);
  uint16_t last_tick = xTaskGetTickCount();

  while (1)
  {
    uint16_t dt = (xTaskGetTickCount() - last_tick) % portMAX_DELAY;
    last_tick = xTaskGetTickCount();
    // ChassisControl_ptr->KinematicAndControlUpdate(dt, imu.getyaw());
    Controller.KinematicAndControlUpdate(dt, ch040.getYaw());
    // 步进不需要速度环，此处仅为了读取电机速度
    // ChassisControl_ptr->MotorUpdate(dt);
    vTaskDelay(2);
  }
}

void HC08CallBack(void *param)
{
  if (HC08Uart.recv_buff[0] == 0xA5 && HC08Uart.recv_buff[HC08_length+2] == 0x5A)
  {

    if (HC08Uart.recv_buff[1] == 0x0C && HC08Uart.recv_buff[HC08_length+1] == 0x0C)
    {
      tar_rad = 0;
    }
    if (HC08Uart.recv_buff[2] == 0x0D && HC08Uart.recv_buff[HC08_length+1] == 0x0D)
    {
      tar_rad = -PI / 2;
    }
    if (HC08Uart.recv_buff[3] == 0x0E && HC08Uart.recv_buff[HC08_length+1] == 0x0E)
    {
      tar_rad = PI;
    }
    if (HC08Uart.recv_buff[4] == 0x0F && HC08Uart.recv_buff[HC08_length+1] == 0x0F)
    {
      tar_rad = PI / 2;
    }
    if (HC08Uart.recv_buff[5] == 0x42 && HC08Uart.recv_buff[HC08_length+1] == 0x42)
    {
      staright_dir = 1;
    }
    if (HC08Uart.recv_buff[6] == 0x5B && HC08Uart.recv_buff[HC08_length+1] == 0x5B)
    {
      staright_dir = -1;
    }
		    if (HC08Uart.recv_buff[7] ==0x4D && HC08Uart.recv_buff[HC08_length+1] ==0X4D )
    {
      Calibration_flag=1;
    }
				    if (HC08Uart.recv_buff[8] ==0x21 && HC08Uart.recv_buff[HC08_length+1] ==0X21 )
    {
     target_upper_loacation=down_location;
    }
						    if (HC08Uart.recv_buff[9] ==0x2C && HC08Uart.recv_buff[HC08_length+1] ==0X2C )
    {
      target_upper_loacation=up_location;
    }
  }
}
void StepCallBack(void *param)
{
  UNUSED(param);
  for (auto i = 0; i < 4; i++)
  {
    if (stepmotor_list_ptr[i] != nullptr)
    {
      stepmotor_list_ptr[i]->UARTCallback(StepMotorUart.recv_buff);
    }
  }
}
void ch040CallBack(void *param)
{
  ch040.analyze_data(ch040Uart.recv_buff);
}

// .............................................'RW#####EEEEEEEEEEEEEEEEEEEEEEEEWW%%%%%%N%%%%%%NW"...........
// ............................................/W%E$$$$EEEE######EEEEEEEEEEEEEEEE%%@NN@@$@@N%%%%N%]~`........
// ........................................i}}I&XIIYYXF&R#E$$$$$EEE##EEEEEEEEEEEE$N$#$K1:!YW@N%%%%@N$KY]+";..
// .....................................!>>li!"~~~'~~~~~!"i/1lIFK#E$$$EEEEEEE$$EEE%I::.....,]E@@@NNN@M$E$R>..
// ....................................+1"""i>"""""!~''''~~~!!~~!>/]Y&#$$$EEEEWWEEE$F,.......:>IRE$#&I/>'....
// ...................................;*lX&NM@@NW$#RFIl1i"!~~"">>!~~~!i}Y&#$W$EW%$EEMi...........::...'l1....
// ]}/+>~,............................,*YRNNNN@@MMMMMMMM@WRF*1>!~"!~!!~~!>+1IK$W%%W%1.................!*+....
// FFF&K&FYYYI]/"'`....................!K%W$$$$$$$EEEEEEE$W%%%WE&I]+!~~~!">"~~i*#%@#...................';....
// }}}}}}]l*XR#$WWERXl/!,:........,>>i/YK&&&&KKKKRR##EE$$$$$EEEE$$$EKYl/>!'~!"!+]IRNI..................'':...
// lllll]]]]}IYYXFK#W%N%$RFl+~`..`X/>>>!~~~~~!!"""">>ii+/}*YXK#EE$$WWWW$#Fl+"'~+**]*FI"................>i....
// ]]]]]]]]]]YXXXXYYXFRE$WW%%W#FlXl;!">+//i">"~'''''~!""!~~~!""i/1]*YFR#$%%WE&l/1]**lI&!.............>]]ll~..
// ]]]]]]]]]*XXXXXXXXYYX&R$$EE$$WWRR#WWWWW$E##KXI*1>!~!!""""!!!~~~'~~!!"+}I&R$NNWKYll*E"............"}/,~I&'.
// ]]]]]]]]lYXXXXXXXXXXXYYXKE$$E#E$$$$$$$$$$$$$$WWW$#X}1+>>""!''''~!!">""""""/]Y#W%$FRY............./+,.~lF>.
// ]]]]]]]]YXXXXXXXXXXXXXXXYYFKEW$E#EEEEEEEEEEEEEEEE$$WWW$$E#RFYl/+i!''!>"!!~!i]]]*XR#1'............!I/!]XI`.
// ]]]]]]]IXXXXXXXXXXXXXXXXXXYYYFE%WEEEEEEEEEEEEEEEEEEEEEEE$$$%%NN%$EKY]+i"!!"ilII*l]lXK/.:..........;+1/>:..
// ]]]]]]IXYXXXXXXXYYYYXXXXXXXYYR$RK$%$EEEEEEEEEEEEEEEEEEEEEEEE##EE$%NNNWE#R&&XI**llll]Y*.......::`,,`:::....
// ]]]]]*XXXXXXXXXXYYYYYYYXXXY&$#I/>/YE%$EEEEEEEEEEEEEEEEEEEEEEEEEEEE#EE$$WW$$W$ER&Y**]&}~+]IFRE$WW%%%%W$$#KX
// ]]]]lYXXXXXXXYYYYYYYYYYXXYK#I/ii+i>lYKWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$$WW$$#@%NMMM@@NNNN%%NNNNNN@@
// ]]]]YXXXXXXYYYYYYYYYYYXYYKX1iiiii+l1>i}KWWE#EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$W%@N%%%%%%%%%%%%%%%%%%%
// ]]]*XXXXXXXXXXXXYYYYYXYY&*++iiii+]+>++>11X$%$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N@@NNNN%%%%%%NNN%%
// ]]]YXXXXXXXYYYYYYXXYXYX&}i+iiiii1+iiii+*>>+*RWWEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE$%N%W$$$$$$$$$WW%%
// ]]*XXYYYYYXX&K#$&YXXXXK}>+iiiii++iiii+FI>+i>>}F$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// }}IYYXFK#E%N%%NEYXXXYK}>iiiiiiiiii+>1I}]>iiii>"+*R$W$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// &XK#EWN@@%#YWN$YYXXY&l>iiiiii++iii>}I"il>iiiiiii"+1IR$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// $NN@N$&}!`.*NWXYXXYFI1/ii++i"!i>+>}l"!i]"iiiiiiii/i>/1IEW$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// "Y&l>,.:~1F@%KYXXXXF}Yi+i"',.';:,1]"+!'/;i">iiii>/i/1"1]IIX#$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ......*@M@%RFIYYXYF1F}!'`::::!`."]""~'!]~"":~"iii/i}/+F***>i*YF#E$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// ......`+FWNWERFXYXl+Y`::::::,".!}~,:.:.~',*:::,'!+/}!/]!>l/"}X>i1lXRE$$$EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE
// .........;/XEN@N%Wi&].`:::`:!',]/1i~,::`>`}>.`::.~1""1;::~1"+]li">>/llXRE$$$$$EE##EEEEEEEEEEEEEEE####EEEE$
// ............:'+lFK}N+:`:::`:!!&%NW$W$&]~,'!/~.``.~!.!''>+/IY>]lX/""11>ii+1lY&#$WWWW$$$$$E$$$$$$$WWW%%%W$#&
// .................>i~~,`:::`,}#M#}"'F%$W$}`;'!;.:`,','1XFK@@@RF@@@&~~~,!>ii>>+i/}lIXF&KR#$R#RRRKK&XIl1i!'`.
// ................`+'.~'`::`:`}$X`.::"&KFK&,:`'~!:`,'`~!;.;X&FKK$&l##l'::`;!>+/+i>>>>>>ii>I!.:..............
// ...:;..`!/:.....;i;."!`::`:;I+~.:.!E&FW#K':'',!"''';...;lYR#K&#K;"1#]~`::,:>i~i++++iii/}++................
// ...;Y.;/Y`......'>;.>+,::`.~E]:::.'K/"l}i`:`:::`;;'!;`.iK/}%&lRI.`1*'``:,,:+"~,,~"i++i1#+},...............
// ...:,>/.......~"'.;l;:``:'Y]'```./+~'';::``::::::`,`:~/~~i"!/'.:i;:`:,,:/+;''.::,'!"**>li...............
// ....:,.;;.......~"':.>i:`:',+l>;'';'";;,``:`:::::::::`,`'~~~~+":`~~```;;'>li!;`::::,.`Xi.1l`..............
// .......'];......`i;;.:I'::'I>1>'~~'';;,,```::::::::```,;;''~~''~~'`,'>>>>'/"~`,::::,`,X".`l+..............
// ........~~......."'~`/>+:::lll";'';;;,,,``:``;~::::```,;;'''~!>>"!!>ii!;:,~,'+!::::`;,Y~..,Y'.....::......
// .......:,.........!~/!.~+`.>]*>:,,,,,,````:`;;;`::````,,,,;;;~!'1/"!;`:::!~+]+"`::`:;;*~...~l.....`>,.....
// ........>..........>i!!++!'`/1Ii`::```````:::::`:::::`````,,,,`'i``:::::;]}/iii,:`::`!*~....>".....`/`....
// ........,`........;>.'>"~.i++]/ll+'`::::::``````::::::````::..,+```:`:`:"+ii+++':`:`:;Fi.....i`.....;+....
// ........`,.......`+:~!.;':i++/+i/}}1+>~;`::.....::::::...:,~i]X~`,:`:`:;/+++++1!:`::`.]I.....`"......+;...
// ........:~......:i,~'..";,ii+/+++++/1}]]}1/i>"!!;,,,,,>}lII**Y>`,:`:;,,/++++/+1+`:::`:,F,.....~,.....'>...
// ................"~'`..:1,,+i1+++//////}111}}}IY$K">>>!*NFl&X]>,,:`.'~`*]++++//+}':`:``.>1.....:".....'i...
// ...............~i':...!1.'+//+++//+/+]l+1]lIF]/Kl"">>+11>"1&i``::`!~;]I]++++//i1i:`::~;./;.....",....!;...
// ..............,1;.....]~.~+}++++/+///*]Y&F$Kl+}1!i++}1+i"11'`:::;!~i*]l]+//+/1++/,:`:~i``/.....'".........
// .............:]~....."}.:"}/+++////i1XRRYF*]lFKY/lI/`;,:"+~!:::,!"l&XYFY++/+/]++/~:`:~+".~i....;/.........
// .............+>.....`*;::/}++++//+/*#El}FIl*F&X]I*IX+`;1}'i::,~>+FYIX&#%#/++1]i++>:`:~++;.+'...~l.........
// ............;}......+!.:'l+++++1+/#N@/'i#F1!1]*"l*I*]+"+l}1+i+i>iRRE$$$ENIi+1]++//;::!++":`i.:.1].........
// ............/~....."i.`.+1i+++/1i*WW&~!1Wi`,+i/]Il>'`:.'I*Y>!>}FE$$EEEE#%*i+}}++1}!:`>+i+,.!~."Ii.........
// ............]'....'/.`;`1+++++//i&EW*~~YF'>+}]1//i"`.:''.Y+iY#$$EEEEEEEEW]i+}/++/*i`,++i/]::>+*l,.........
// ...........`l"...`/`:"~'/}/+++//YWEW*!+*+>iiI]]/">>i;,!`.lX$$EEEEEEEEEEEWN*+1++++Y1;'+i+i*+.~l}~..........
// ...........`l/,..+;.,+'~]*+//+1EW$E$X+1"""iY&i1l>"""ii+`,EWEEEEEEEEEEEEEE$W1++++i*]'>+i++//~.+;...........
// ............]}/`"".:!+~!I}+//iY%#$$EX1~""iFI*~i1]"">!/]`"%EEEEEEEEEEEEEEE$$1i++++/1"+++++};".!;...........
// ............'Y/1+.`:>+>/]1+//iXW#E$$**X>+F*/";">]l!"+&%I+WE$$$$$$$E$$WW%N%}i++++/>i+i++++]:;;,>...........
// .............~l}`::;i+i]i}+/+l@%$$E$&XYYY}!1,.!i>XIYX#N$REWEEEEEEEEEEEEE$N1+/+++/'iiiiii/1.`>:+...........
// ..............,i::;}i+i],*+++$@$EE$E%&']!~"+;~;>!"*}>$$$$EEE$EEEEEEEEEEE#%Fi//+1~'/i+++il>..>`i`..........
// ..............';:./]i++}.}li]NEEEE$W$}:i]+i!;~;i>i>,>%F/*$EEWWEEEEEEEEEEEE%Xi+//./+i++++*`..";i`..........