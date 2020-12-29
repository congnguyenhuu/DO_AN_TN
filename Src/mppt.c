#include "mppt.h"
//thuat toan mppt
//TIM_HandleTypeDef htim1;
//float vinmin=35,vinmax=43,vout=56.4;
float vinmin=17,vinmax=22,vout=28.2;
float dutymax=0.4;
float dutymin=0.1;
float deltaduty=0.001;
float duty=0.5;
float vk_1 = 0;
float ik_1 = 0;
float pk_1 = 0;
float duty_1 =0.5;
float deltaV;
float deltaI;
float deltaP;
float pk;
void MPPT_INIT(float vk,float ik)
{
	dutymax=1-vinmin/vout;
	dutymin=1-vinmax/vout;
	duty=dutymin;
	duty_1=dutymin;
	vk_1 = vk;
  ik_1 = ik;
  pk = vk * ik;
  pk_1 = pk;
}
float MPPT_INC(float vk,float ik)
{
 deltaV = vk - vk_1;
 deltaI = ik - ik_1;
 pk = vk * ik;
 deltaP = pk - pk_1;
//thuat toan INC
if (deltaV==0)
{
	if (deltaI == 0) duty = duty_1;
	else
	{
		if (deltaI>0) duty=duty_1 + deltaduty; ////
		else duty=duty_1 - deltaduty; /////
	}
}
else
{
	if (deltaI/deltaV == - ik/vk) duty = duty_1;
	else
	{
		if (deltaI/deltaV > - ik/vk) duty=duty_1 - deltaduty;
		else duty=duty_1 + deltaduty;
	}
}
//gioi han duty
if (duty > dutymax) duty = dutymax;
else if (duty < dutymin) duty = dutymin;
duty_1 = duty;
vk_1 = vk;
ik_1 = ik;
pk_1 = pk;
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty*1439);
return duty;
}


float MPPT_PO(float vk,float ik)
{
  deltaV = vk - vk_1;
  deltaI = ik - ik_1;
  pk = vk * ik;
  deltaP = pk - pk_1;
//P&O
if (deltaP !=0 )
{
	if (deltaP > 0)
		{
			if (deltaV >0) duty=duty_1 - deltaduty;
			else duty=duty_1 + deltaduty;
		}
	else
		{
			if (deltaV >0) duty=duty_1 + deltaduty;
			else duty=duty_1- deltaduty;
		}
}
else duty= duty_1;

//gioi han duty
if (duty > dutymax) duty = dutymax;
else if (duty < dutymin) duty = dutymin;
duty_1 = duty;
vk_1 = vk;
ik_1 = ik;
pk_1 = pk;
//	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,duty*1439);
return duty;
}
