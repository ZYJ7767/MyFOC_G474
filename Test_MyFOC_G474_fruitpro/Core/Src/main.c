/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Led_Key.h"
#include "Encoder.h"
#include "Foc_Function.h"
#include "Observer.h"
#include "Filter.h"
#include "dwt_timer.h"
#include "math.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint16_t  ADC_Vbus   = 0;           //ADC原始值
uint16_t  ADC_IsensU = 0;
uint16_t  ADC_IsensV = 0;
uint16_t  ADC_IsensW = 0;

float   Vbus         = 0;           ////电压电流真实值
float   IsensU_raw   = 0;           //电流生值
float   IsensV_raw   = 0;
float   IsensW_raw   = 0;
float   IsensU       = 0;           //电流滤波值
float   IsensV       = 0;
float   IsensW       = 0;

uint8_t Offset_Flag  = 0;           //偏置校准变量
uint8_t Run_Flag     = 0;           //电机启动标志
uint8_t Run_Switch   = 0;           //观测器切换标志为
uint8_t ThetaUp_Flag = 0;           //开环位置自增标志


uint16_t U_Offset    = 0;           //偏置ADC值
uint16_t V_Offset    = 0;
uint16_t W_Offset    = 0;

#define  OC_TRIP_A   18.0f          // 电流保护
volatile uint8_t g_fault_oc = 0;   // 过流闩锁，触发后保持故障态


uint8_t key;                        //按键值
float   my_theta      = 0;          //开环位置
float   my_step       = 0.010f;     //开环增量
float   my_add        = 0.0f;

uint16_t EncodeValue    = 0;        //编码器当前值
float   encode_e_theta  = 0;        //真实电角度
float   encode_speed    = 0;        //真实速度
uint16_t encode_cnt     = 0;        //编码器计圈

float   smo_e_theta     = 0;        //SMO_Atan
float   smo_pll_e_theta = 0;        //SMO_PLL
float   bef_e_theta     = 0;        //
float   smo_speed_raw   = 0;
float   smo_speed       = 0;        //SMO_RPM

float   smo_err         = 0;        //观测器和实际角度误差

float   final_theta     = 0;        //IF/SMO blend output angle

float    Ud;
float    Uq;
float    Ualpha;
float    Ubeta;
float    Id;
float    Iq;
float    Ialpha;
float    Ibeta;

float    Vup        = 0;            //VF开环频率（速度）变量
float    Uup        = 0;            //VF开环电压变量

float    kp         = 0.2274f;      //电流环PID调试变量 200Hz带宽
float    ki         = 0.0148f;      //电流环PID调试变量
float    skp        = 0.016f;       //速度环PID调试变量
float    ski        = 0.00060f;     //速度环PID调试变量

float    Iqref_start= 4.0f;         //IF-SMO切换电流环
float    Iqref      = 4.0f;         //Iq参考
float    Speedref   = 0.0f;

float    SMO_K      = 3.0f;         //初始滑膜增益
float    Kr         = 0.0f;         //谐振增益


LPF1_t g_smo_speed_lpf  = {0};      //低通滤波器
DWT_Time_t t = {0};                 //DWT时间戳


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */
  DWT_Timer_Init();
  
  /******* 关闭所有led灯 *******/
  LED0(1);
  LED1(1);
  
  /******* 控制器参数赋值 *******/
  C_PI.Kp  = kp;                                               //wb*Ls = 1000*0.000444463 = 0.444463
  C_PI.Ki  = ki;                                               //wb*Rs = 1000*0.223025     = 223
  S_PI.Kp  = skp;
  S_PI.Ki  = ski;

  PR_Id.Kr = Kr;
  PR_Iq.Kr = Kr;
  
  /*********** ADC相电流采集 ***********/
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);                     //在启动新一轮ADC转换前，清除之前的转换结束标志
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);                      //End of Conversion标志位

  HAL_ADCEx_Calibration_Start(&hadc1 , ADC_SINGLE_ENDED);           //ADC自校准
  HAL_ADCEx_InjectedStart_IT(&hadc1);                               //开启三相电流值ADC采集以及ADC中断

  /*********TIM1&6&3 PWM互补输出 触发 定时 编码器**********/
//  HAL_TIM_Base_Start_IT(&htim6);                                  //TIM6 计时
  
  HAL_TIM_Encoder_Stop(&htim3, TIM_CHANNEL_ALL);                    //TIM3 编码器 先停止
  __HAL_TIM_SET_COUNTER(&htim3, 0);                                 //清0计数器
  
  HAL_TIM_Base_Start( &htim1);                                      //TIM1 PWM
  HAL_TIM_PWM_Start ( &htim1, TIM_CHANNEL_4);                       //触发通道4这里用的这个事件触发ADC采集
  
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1);                        //三相PWM通道开启
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3);
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1);                     //三相PWM互补通道开启
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2);  
  HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3); 

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 35;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*********************************** 各类中断存放区 主要功能实现 ***********************************/

/*ADC三相电流低端采样完成中断 FOC运算 10kHz */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)                //10kHz频率 0.0001s算一次
{

    DWT_Timer_Start(&t);
    static uint8_t  offset_cnt   = 0;                                           // 偏置测量计数
    
    static uint16_t align_cnt    = 0;                                           // 0位对齐计时
    static uint32_t openloop_cnt = 0;                                           // 开环计时
    
    static uint16_t smo_settle_cnt = 0;                                         // 进入SMO_ONLY后稳定计数
    static uint8_t  speed_loop_en  = 0;                                         // 速度环使能标志
    static uint8_t  speed_cnt      = 0;                                         // 速度环分频计数（10kHz->1kHz）
    static float    speedref_cmd   = 0.0f;                                      // 速度给定斜坡
    
    
    smo_pll_e_theta = SMO_PLL_Update(&SMO, &PLL, MyFoc.Ualpha, MyFoc.Ubeta, MyFoc.Ialpha, MyFoc.Ibeta);
    smo_speed_raw   = (float)PLL.Est_RPM;                                                                         //Jlink调试
    smo_speed       = LPF1_Update(&g_smo_speed_lpf, smo_speed_raw, 0.015f);
    MyFoc.speed     = smo_speed;
    //my_add          = SMO_GetPhaseComp(smo_speed);                              //相位补偿
    SMO_K = 0.0082f * smo_speed + 0.2133f;                                      //动态滑膜增益
    if(SMO_K < 1.5f)  SMO_K = 1.5f;
    else if(SMO_K > 15.0f) SMO_K = 15.0f;
    SMO.K = SMO_K;
    
    UNUSED(hadc);
    if(hadc == &hadc1)
    {

        if(Offset_Flag == 0)
        {
            offset_cnt++;
            ADC_IsensU = hadc1.Instance->JDR1;
            ADC_IsensV = hadc1.Instance->JDR2;
            ADC_IsensW = hadc1.Instance->JDR3;
            
            U_Offset += ADC_IsensU;
            V_Offset += ADC_IsensV;
            W_Offset += ADC_IsensW;

            if(offset_cnt >= 10)
            {
                U_Offset = U_Offset/10;
                V_Offset = V_Offset/10;
                W_Offset = W_Offset/10;
                Offset_Flag  = 1;                                               //偏置完成 标志位置1
                Run_Flag     = 1;                                               //允许启动
                
            }
        }
        else
        {
            ADC_IsensU = hadc1.Instance->JDR1;
            ADC_IsensV = hadc1.Instance->JDR2;
            ADC_IsensW = hadc1.Instance->JDR3;
            
            IsensU = - (float)(ADC_IsensU - U_Offset)* 0.00785664f;             ////取负因为硬件问题，电流和采集对应关系相反了
            IsensV = - (float)(ADC_IsensV - V_Offset)* 0.00785664f;
            IsensW = - (float)(ADC_IsensW - W_Offset)* 0.00785664f;
            
            //过流保护
            if ((fabsf(IsensU) > OC_TRIP_A) ||(fabsf(IsensV) > OC_TRIP_A) ||(fabsf(IsensW) > OC_TRIP_A))
            {
                g_fault_oc = 1;
            }
            if (g_fault_oc)
            {
                Run_Flag = 0;  
                Motor_SoftwareTrip_StopAll();
                speedref_cmd   = 0.0f; // 把中断里的静态参考也归零
                speed_loop_en  = 0;
                speed_cnt      = 0;
                smo_settle_cnt = 0;
                openloop_cnt   = 0;
                align_cnt      = 0; 
                return;
            }
            //电机停机/启动操作
            if (Speedref == 0.0f && Run_Flag == 1)
            {
                Run_Flag = 0;                 // 关闭主运算FOC标志位
                Motor_SoftwareTrip_StopAll(); // 关闭PWM、清空所有PID积分
                
                speed_loop_en  = 0;           // 将运行强拖与斜坡的所有状态清零，以备下次干净的启动
                speed_cnt      = 0;
                smo_settle_cnt = 0;
                openloop_cnt   = 0;
                align_cnt      = 0; 
                speedref_cmd   = 0.0f;
                my_theta       = 0.0f;
                Blend.state    = BLEND_STATE_IF_ONLY;
                return;                       // 本次中断提前结束
            }
            else if (Speedref > 0.0f && Run_Flag == 0 && g_fault_oc == 0)
            {
                my_step = 0.010f;             //恢复初始步长
                openloop_cnt   = 0;
                align_cnt      = 0; 
                smo_settle_cnt = 0;
                speed_loop_en  = 0;
                my_theta       = 0.0f;
                speedref_cmd   = 0.0f;
                MyFoc.speed    = 0.0f;
                Iqref          = Iqref_start;
                Blend.state    = BLEND_STATE_IF_ONLY;
                
                HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_1);// 重启定时器1的6个PWM输出通道
                HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_2);
                HAL_TIM_PWM_Start( &htim1, TIM_CHANNEL_3);
                HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_1);
                HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_2);  
                HAL_TIMEx_PWMN_Start( &htim1, TIM_CHANNEL_3); 
                
                Run_Flag = 1;                 // 重置运行标志位，放行后续的 IF 开环拖动控制代码
            }
            
        }
     }
     if(Run_Flag)
     {
         float I_ForwardAdd = 5.0f;  //静摩擦补偿前馈量  
         
         if(align_cnt < 2000)  //0位对齐
         {
                align_cnt++;
                my_theta = 0.0f; 
                IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref_start + I_ForwardAdd , 4.712389f);
         }
         else
         {
                // 1) IF开环角度推进
                my_theta += my_step;
                my_theta  = Normalize_theta(my_theta);

                // 2) IF/SMO加权融合角度
                final_theta = IF_SMO_Blend(&Blend, my_theta, smo_pll_e_theta, openloop_cnt);

                if (Blend.state != BLEND_STATE_SMO_ONLY)
                {
                    if (openloop_cnt > 10000)
                    {
                        I_ForwardAdd = 0;
                    }
                    
                    // 仍在IF阶段（含CONVERGING/BLENDING）：只跑电流环，给固定Iqref_start
                    IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref_start + I_ForwardAdd, final_theta);//final_theta
                    openloop_cnt++;

                    // 离开SMO_ONLY后，速度环状态全部清零，避免脏状态
                    speed_loop_en     = 0;
                    smo_settle_cnt    = 0;
                    speed_cnt         = 0;
                    speedref_cmd      = 0.0f;
                    S_PI.speed_KI_sum = 0.0f;
                    Iqref             = Iqref_start;
                }
                else
                {
                    // 已进入SMO_ONLY
                    if (speed_loop_en == 0)
                    {
                        // 3) 先仅SMO+电流环稳定一段时间
                        SMO_C_Control(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref_start, Normalize_theta(final_theta + my_add));

                        // 锁相误差满足门限才累计，防止没锁稳就开速度环
                        if (fabsf(PLL.Err) < 0.6f)
                        {
                            smo_settle_cnt++;
                        }
                        else
                        {
                            smo_settle_cnt = 0;
                        }

                        // 1200个10kHz周期 = 120ms
                        if (smo_settle_cnt >= 800)
                        {
                            // 4) 切入速度环，做无扰预置
                            speed_loop_en = 1;
                            speed_cnt     = 0;

                            speedref_cmd   = MyFoc.speed;      // 从当前速度起步
                            S_PI.speed_ref = speedref_cmd;
                            S_PI.err_speed = S_PI.speed_ref - MyFoc.speed;

                            // bumpless: 让速度环输出初值接近Iqref_start
                            if (S_PI.Ki > 1e-6f)
                            {
                                S_PI.speed_KI_sum = (Iqref_start - S_PI.Kp * S_PI.err_speed) / S_PI.Ki;
                            }
                            else
                            {
                                S_PI.speed_KI_sum = 0.0f;
                            }

                            // 防止预置过大
                            if (S_PI.speed_KI_sum >  12000.0f) S_PI.speed_KI_sum =  12000.0f;
                            if (S_PI.speed_KI_sum < -12000.0f) S_PI.speed_KI_sum = -12000.0f;

                            Iqref = Iqref_start;
                        }
                    }
                    else
                    {
                        // 5) 速度环分频执行（10kHz -> 1kHz）
                        speed_cnt++;
                        if (speed_cnt >= 10)
                        {
                            speed_cnt = 0;

                            // 速度给定斜坡，避免给定跳变
                            if (speedref_cmd < Speedref)
                            {
                                speedref_cmd += 0.5f;
                                if (speedref_cmd > Speedref) speedref_cmd = Speedref;
                            }
                            else if (speedref_cmd > Speedref)
                            {
                                speedref_cmd -= 0.5f;
                                if (speedref_cmd < Speedref) speedref_cmd = Speedref;
                            }

                            S_PI.speed_ref = speedref_cmd;
                            SpeedPI(&MyFoc, &S_PI, &Iqref);
                        }
                        // 6) SMO角度下执行电流环，Iqref由速度环给出
                        SMO_C_Control(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, Normalize_theta(final_theta + my_add));
                    }
                }
         }
     }

    DWT_Timer_Stop(&t);


    //jlink调试
    Ud      = MyFoc.Ud;
    Uq      = MyFoc.Uq;
    Ualpha  = MyFoc.Ualpha;
    Ubeta   = MyFoc.Ubeta;
    Id      = MyFoc.Id;
    Iq      = MyFoc.Iq;
    Ialpha  = MyFoc.Ialpha;
    Ibeta   = MyFoc.Ibeta; 
    
}//ADC采样完成中断




/************** 一阶低通数字滤波 **************/ 
float LowPassFilter(float input , float a)                                       //a为滤波系数
{
    static float prev_output = 0;                                                //静态变量保存上一次输出    
    static uint8_t init_flag = 0;                                                //初始标志位，防止畸变
    
     if(!init_flag) 
     {
        prev_output = input;
        init_flag = 1;
        return prev_output;
     }
    
    float output = a * input + (1.0f - a) * prev_output;
    prev_output = output;
    return output;
}

/************** 停机功能函数 **************/ 
void Motor_SoftwareTrip_StopAll(void)
{
    // 停各通道
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);

    // 参考值清零
    Iqref    = 0.0f;
    Speedref = 0.0f;
    my_step  = 0.0f;

    C_PI.Id_ref = 0.0f;
    C_PI.Iq_ref = 0.0f;
    S_PI.speed_ref = 0.0f;

    // 积分清零
    C_PI.Id_KI_sum = 0.0f;
    C_PI.Iq_KI_sum = 0.0f;
    S_PI.speed_KI_sum = 0.0f;
}





/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
