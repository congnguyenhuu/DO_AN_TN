#ifndef _mppt_h_
#define _mppt_h_

#include <main.h>
void MPPT_INIT(float vk,float ik);
float MPPT_INC(float vk,float ik);
float MPPT_PO(float vk,float ik);

#endif