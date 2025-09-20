#ifndef HC_SR04_H
#define HC_SR04_H
#include "main.h"
#include "stdint.h"

typedef struct HC_SR04
{
    uint32_t Distance_m;    // 距离，单位米
    uint32_t Distance_mm; // 距离，单位毫米
    GPIO_TypeDef *Trig_port;
    GPIO_TypeDef *Echo_port;
    uint16_t Trig_pin;
    uint16_t Echo_pin;
    uint64_t time;
    uint64_t time_end;
__IO	int shit_flag;
__IO	  int enable_flag;
__IO	int finish_flag;
__IO	int round;
} HC_SR04_t;

void HC_SR04_IRQ(HC_SR04_t * _hc_sr04);
void HC_SR04_get_distance(HC_SR04_t * _hc_sr04);
void HC_SR04_Time_Plus(HC_SR04_t * _hc_sr04);
extern HC_SR04_t right_hc_sr04;
#endif
