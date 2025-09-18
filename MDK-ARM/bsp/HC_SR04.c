#include "HC_SR04.h"
HC_SR04_t right_hc_sr04=
{
.Trig_port=GPIOE,
	.Echo_port=GPIOE,
.Trig_pin=GPIO_PIN_3,
.Echo_pin=GPIO_PIN_2,
.enable_flag=0,
.finish_flag=1,
.shit_flag=0,
};
static void sonar_mm_trig(HC_SR04_t * _hc_sr04)									//测距并返回单位为毫米的距离结果
{

HC_SR04_t * _it=_hc_sr04;
	switch(_it->round)
	{
		case 0:
		{
_it->Distance_m=0;
_it->Distance_mm=0;
HAL_GPIO_WritePin(_it->Trig_port,_it->Trig_pin,1);  			
			_it->round++;
		break;
		}
		case 1:
		{
		_it->round++;
		break;
		}
		case 2:
		{
			_it->round++;
			break;
		}
		case 3:
		{
			_it->round++;
		break;
		}		
		case 4:
		{
		HAL_GPIO_WritePin(_it->Trig_port,_it->Trig_pin,0);
			_it->shit_flag=1;
			_it->enable_flag=0;		
			_it->round=0;
		break;
		}
		default:
			break;
	}
 
}

static void sonar_mm_echo(HC_SR04_t * _hc_sr04)	
{
	uint32_t Distance= 0;
HC_SR04_t * _it=_hc_sr04;	
while(HAL_GPIO_ReadPin(_it->Echo_port,_it->Echo_pin)==0);
_it->time=0;
while(HAL_GPIO_ReadPin(_it->Echo_port,_it->Echo_pin)==1);
	_it->time_end=_it->time;
	if(_it->time_end/100<38)
	{
		Distance=(_it->time_end*346)/2;
		_it->Distance_mm=Distance/100;
		_it->Distance_m=_it->Distance_mm/1000;
	}
	_it->shit_flag=0;
	_it->finish_flag=1;
}

///下面函数是外部接口
//放中断里

void HC_SR04_IRQ(HC_SR04_t * _hc_sr04)
{
HC_SR04_t * _it=_hc_sr04;	
if(_it->enable_flag==1&&_it->finish_flag==1)
{
//	_it->finish_flag=0;
sonar_mm_trig(_it);
}
if(_it->shit_flag==1)
{
sonar_mm_echo(_it);
}

}
void HC_SR04_Time_Plus(HC_SR04_t * _hc_sr04)
{
_hc_sr04->time++;

}

void HC_SR04_get_distance(HC_SR04_t * _hc_sr04)
{
_hc_sr04->enable_flag=1;
	
}

