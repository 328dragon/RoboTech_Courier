#include "SR04.h"
#include "bsp_tim_control.h"

void SR04_Register(SR04_t *SR04, GPIO_TypeDef *Trig_Port, uint16_t Trig_Pin,
				   TIM_HandleTypeDef *Echo_htim, uint32_t Echo_TIM_Channel)
{
	SR04->Ttig_Port = Trig_Port;			   // 触发端口
	SR04->Trig_Pin = Trig_Pin;				   // 触发引脚
	SR04->Echo_htim = Echo_htim;			   // 定时器句柄
	SR04->Echo_TIM_Channel = Echo_TIM_Channel; // 定时器通道
	SR04->distant = 0.0f;					   // 测量距离
	SR04->measure_Cnt = 0;					   // 状态标志位
	SR04->high_time = 0;					   // 超声波模块返回的高电平时间
}

void SR04_Elapsed_callback(SR04_t *_SR04)
{
	switch (_SR04->measure_Cnt)
	{
	case 1:
	{
		HAL_GPIO_WritePin(_SR04->Ttig_Port, _SR04->Trig_Pin, 1);
			_SR04->measure_Cnt += delay_no_conflict(&_SR04->delay_temp_count, 3); // 延时30us
		break;
	}
	case 2:
	{
		HAL_GPIO_WritePin(_SR04->Ttig_Port, _SR04->Trig_Pin, 0);
		_SR04->measure_Cnt++;
		__HAL_TIM_SET_CAPTUREPOLARITY(_SR04->Echo_htim, _SR04->Echo_TIM_Channel, TIM_INPUTCHANNELPOLARITY_RISING); // 设置为上降沿捕获
		HAL_TIM_IC_Start_IT(_SR04->Echo_htim, _SR04->Echo_TIM_Channel);											   // 启动输入捕获
		break;
	}
	case 5:
	{
		_SR04->high_time = _SR04->measure_Buf[1] - _SR04->measure_Buf[0]; // 高电平时间
		_SR04->distant = (_SR04->high_time * 0.034) / 2; // 单位cm
		_SR04->measure_Cnt = 0; // 清空标志位
__HAL_TIM_SET_COUNTER(_SR04->Echo_htim,0);
	
		break;
	}
	default:
		break;
	}
}
void SR04_Echo_IC_callback(SR04_t *_SR04)
{
		switch(_SR04->measure_Cnt){
			case 3:
				_SR04->measure_Buf[0] = HAL_TIM_ReadCapturedValue(_SR04->Echo_htim,_SR04->Echo_TIM_Channel);//获取当前的捕获值.
				__HAL_TIM_SET_CAPTUREPOLARITY(_SR04->Echo_htim,_SR04->Echo_TIM_Channel,TIM_ICPOLARITY_FALLING);  //设置为下降沿捕获
				_SR04->measure_Cnt++;
				break;
			case 4:
				_SR04->measure_Buf[1] = HAL_TIM_ReadCapturedValue(_SR04->Echo_htim,_SR04->Echo_TIM_Channel);//获取当前的捕获值.
				HAL_TIM_IC_Stop_IT(_SR04->Echo_htim,_SR04->Echo_TIM_Channel); //停止捕获   或者: __HAL_TIM_DISABLE(&htim5);
				_SR04->measure_Cnt++;
		}


}


void SR04_GetData(SR04_t *_SR04)
{
	_SR04->measure_Cnt=1;
}
