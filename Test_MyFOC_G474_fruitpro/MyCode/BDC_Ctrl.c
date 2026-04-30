#include "BDC_Ctrl.h"


extern TIM_HandleTypeDef htim8; 

/**
 * @brief       直流电机业务初始化
 * @note        假定CubeMX已经将GPIO和TIM8初始化完毕。这里主要负责上电时的安全状态。
 */
void BDC_Init(void)
{
    HAL_TIM_Base_Start( &htim8);
    __HAL_TIM_MOE_ENABLE(&htim8);
    BDC_Stop();
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET);    /*  拉高Shutdown引脚，使能电机驱动芯片 */ 
}

/**
 * @brief       直流电机正转
 * @note        关闭互补通道(CH1N), 开启主通道(CH1)输出PWM
 */
void BDC_Down_drive(void)
{
    
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); 
    /* 必须先关掉另一侧的桥臂，防止直通短路 */
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    /* 开启主通道PWM输出 */
    HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
}

/**
 * @brief       直流电机反转
 * @note        关闭主通道(CH1), 开启互补通道(CH1N)输出PWM
 */
void BDC_Up_drive(void)
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_SET); 
    /* 必须先关掉另一侧的桥臂，防止直通短路 */
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    /* 开启互补通道PWM输出 */
    HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1);
}

/**
 * @brief       直流电机停止
 * @note        同时关闭主通道和互补通道输出
 */
void BDC_Stop(void)
{
    HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_11, GPIO_PIN_RESET); 
}

/**
 * @brief       直流电机设置速度
 * @param       speed: 速度值(最大值14000)
 * @note        主通道直接赋占空比
 */
void BDC_SetSpeed(uint16_t speed)
{
    // 限制最大死区/越界，保障安全
    if(speed > 14000) speed = 14000;
    __HAL_TIM_SetCompare(&htim8, TIM_CHANNEL_1, speed);
}



