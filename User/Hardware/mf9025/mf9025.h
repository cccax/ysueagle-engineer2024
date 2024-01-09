#ifndef _MF9025_H
#define _MF9025_H

#include "stdint.h"
#include "can.h"

#define motor_measure_MF9025(ptr, data,id)													    \
    {																																		\
        (ptr)->last_ecd = (ptr)->ecd;																		\
				(ptr)->ecd = (uint16_t)((data)[7] << 8 | (data)[6]);						\
        (ptr)->speed = -(int16_t)((data)[5] << 8 | (data)[4]);					\
        (ptr)->real_current = -(int16_t)((data)[3] << 8 | (data)[2]);	  \
        (ptr)->temperate = (data)[1];																		\
				if(id == 2) 																										\
				{																																\
					if((ptr)->ecd > 50000)    (ptr)->relativeEcd = (ptr)->ecd - 65535; \
					else (ptr)->relativeEcd = (ptr)->ecd;   											\
				}																														    \
				else if(id == 1)   																			        \
				{																														    \
					if((ptr)->ecd < 15535)    (ptr)->relativeEcd = (ptr)->ecd ;   \
					else (ptr)->relativeEcd = (ptr)->ecd - 65535;   						  \
				}																													    	\
    }                                                                   \
void MF9025_Ecd1_CtrlAll(CAN_TypeDef* CANx ,float angleControl);
void MF9025_Motor_Ecd3Ctrl(CAN_TypeDef* CANx , uint8_t spinDirection , uint16_t angleControl);
void MF9025_Motor_Ecd1Ctrl(CAN_TypeDef* CANx , uint8_t id , float angleControl);
void MF9025_Motor_Ecd_SetZero(CAN_TypeDef* CANx , uint8_t id );
void MF9025_Motor_Pid_Set(CAN_TypeDef* CANx , uint8_t id);
void MF9025_Motor_Stop(CAN_TypeDef* CANx , uint8_t id);
void MF9025_Motor_Resume(CAN_TypeDef* CANx , uint8_t id);
void MF9025_CurrentSendQueue(CAN_TypeDef* CANx , uint8_t id, int16_t iqControl);
#endif
