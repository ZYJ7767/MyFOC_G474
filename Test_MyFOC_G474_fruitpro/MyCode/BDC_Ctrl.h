#ifndef __BDC_MOTOR_H
#define __BDC_MOTOR_H

#include "main.h"
#include "tim.h"

/* 函数声明 */
void BDC_Init(void);                     /* 电机初始状态配置 */
void BDC_Stop(void);                     /* 电机停止 */
void BDC_Down_drive(void);               /* 电机正转 */
void BDC_Up_drive(void);                 /* 电机反转 */
void BDC_SetSpeed(uint16_t speed);  /* speed这里为ccr，越大速度为快 */








#endif /* __BDC_MOTOR_H */
