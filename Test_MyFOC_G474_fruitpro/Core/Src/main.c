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
#include "adc.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Led_Key.h"
#include "Encoder.h"
#include "Foc_Function.h"
#include "Observer.h"
#include "Filter.h"
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

uint8_t key;                        //按键值
float   my_theta      = 0;          //开环位置
float   my_step       = 0.02;       //开环增量

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

float    kp         = 0.725f;       //电流环PID调试变量0.725    
float    ki         = 0.0707f;      //电流环PID调试变量.0707  
float    skp        = 0.008f;       //速度环PID调试变量
float    ski        = 0.00055f;     //速度环PID调试变量

float    Iqref_start= 6.0f;         //IF-SMO切换电流环
float    Iqref      = 6.0f;         //Iq
float    Speedref   = 600.0f;
float    Speedref_cmd   = 0.0f;

LPF1_t g_smo_speed_lpf  = {0};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM3_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  
  /******* 关闭所有led灯 *******/
  LED0(1);
  LED1(1);
  /*********** ADC相电流采集 ***********/
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_JEOC);                     //在启动新一轮ADC转换前，清除之前的转换结束标志
  __HAL_ADC_CLEAR_FLAG( &hadc1, ADC_FLAG_EOC);                      //End of Conversion标志位

  HAL_ADCEx_Calibration_Start(&hadc1 , ADC_SINGLE_ENDED);           //ADC自校准
  HAL_ADCEx_InjectedStart_IT(&hadc1);                               //开启三相电流值ADC采集以及ADC中断

  /*********TIM1&6&3 PWM互补输出 触发 定时 编码器**********/
  HAL_TIM_Base_Start_IT(&htim6);                                    //TIM6 计时
  
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

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    //PI调试
      C_PI.Kp     = kp;                                               //wb*Ls = 1000*0.000444463 = 0.444463
      C_PI.Ki     = ki;                                               //wb*Rs = 1000*0.223025     = 223
//      C_PI.Iq_ref = Iqref;
      
      S_PI.Kp     = skp;
      S_PI.Ki     = ski;

    /***** 软件触发电源电压采集 *****/
      HAL_ADC_Start(&hadc1);
      ADC_Vbus   = HAL_ADC_GetValue(&hadc1);
      Vbus = (float)ADC_Vbus * 0.0176757477f;
      
    /****** KEY_LED 预留模块 ******/
//      key = key_scan(0);
//      if (key)
//      {
//        switch (key)
//        {
//            case KEY0_PRES:
//                LED0_TOGGLE(); 
//                Iqref += 0.5f;
//                if(Iqref >= 10.0f) Iqref = 10.0f;
//                break;

//            case KEY1_PRES:
//                LED1_TOGGLE();
//                Iqref -= 0.5f;
//                if(Iqref <= -10.0f) Iqref = -10.0f;
//                break;

//            case KEY2_PRES:
//                LED0_TOGGLE();
//                LED1_TOGGLE();
//                Iqref = 0.0f;
//                break;
//            default : break;
//        }
//      }
//      else
//      {
//        HAL_Delay(10);
//      }
      
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

/*▲▲▲ADC三相电流低端采样完成中断 FOC运算 10kHz▲▲▲ */
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)                //10kHz频率 0.0001s算一次
{
    //static uint8_t  Still_Flag = 0;                                             //拉到0位标志
    //static uint16_t Still_cnt = 0;                                              //拉到0位计时
    //static uint8_t  err_cnt    = 0;                                             //记录观测角度可接受误差的次数
    //static uint8_t  close_flag = 0;                                             //可以切入闭环标志位
    static uint8_t  offset_cnt   = 0;                                           //偏置测量计数
    static uint16_t openloop_cnt = 0;                                           //开环计时
    
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
            
        }
     }
     if(Run_Flag)
     {
         static uint8_t  speed_div_cnt   = 0;
         static uint16_t smo_only_cnt    = 0;
         static uint8_t  speed_loop_en   = 0;
         
         if(Run_Switch == 0)
         {
         /******* ▲自定义角度 IF强拖启动 *******/
            my_theta += my_step;
            my_theta  = Normalize_theta(my_theta);

            final_theta = IF_SMO_Blend(&Blend, my_theta, smo_pll_e_theta, openloop_cnt);

            if (Blend.state == BLEND_STATE_SMO_ONLY)
            {
                if (speed_loop_en == 0)
                {
                    SMO_C_Control(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref_start, final_theta);

                    smo_only_cnt++;

                    if (smo_only_cnt >= 0) //
                    {
                        speed_loop_en      = 1;
                        speed_div_cnt      = 0;
                        S_PI.speed_KI_sum  = 0.0f;
                        Iqref              = Iqref_start;
                        Speedref_cmd       = MyFoc.speed;
                    }
                }
                else
                {
                    speed_div_cnt++;

                    if (speed_div_cnt >= 10)
                    {
                        speed_div_cnt = 0;
                        
                    if (Speedref_cmd < Speedref)
                    {
                        Speedref_cmd += 3.0f;
                        if (Speedref_cmd > Speedref) Speedref_cmd = Speedref;
                    }
                    else if (Speedref_cmd > Speedref)
                    {
                        Speedref_cmd -= 3.0f;
                        if (Speedref_cmd < Speedref) Speedref_cmd = Speedref;
                    }
                        
                        S_PI.speed_ref = Speedref_cmd;
                        SpeedPI(&MyFoc, &S_PI, &Iqref);
                    }

                    SMO_C_Control(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, final_theta );
                }
            }
            else
            {
                speed_loop_en     = 0;
                speed_div_cnt     = 0;
                smo_only_cnt      = 0;
                S_PI.speed_KI_sum = 0.0f;
                Iqref             = Iqref_start;
                Speedref_cmd      = 0.0f;

                IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref_start, final_theta);
                openloop_cnt++;
            }
        
//            if(Blend.state == BLEND_STATE_SMO_ONLY)
//            {
//            /******* SMO 无感驱动 *******/
//                SMO_C_Control(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, final_theta);
//            }
//            else
//            {
//            /******* IF 强拖驱动 / IF加权角度强拖驱动 *******/
//                IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, final_theta);
//                openloop_cnt++;
//            }
            
            
         }
     }
     
     
     //保持观测
     smo_pll_e_theta = SMO_PLL_Update(&SMO, &PLL, MyFoc.Ualpha, MyFoc.Ubeta, MyFoc.Ialpha, MyFoc.Ibeta);
     smo_speed_raw   = (float)PLL.Est_RPM;                                                                         //Jlink调试
     smo_speed       = LPF1_Update(&g_smo_speed_lpf, smo_speed_raw, 0.02f);
     MyFoc.speed     = smo_speed;
     

//        smo_e_theta     = SMO_Update(&SMO, MyFoc.Ualpha, MyFoc.Ubeta, MyFoc.Ialpha, MyFoc.Ibeta); //+ 0.25f;
//        IF_OpenLoop(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, my_theta);
//        smo_e_theta = SMO_Observer(&SMO, &Mo, MyFoc.Ualpha, MyFoc.Ubeta, MyFoc.Ialpha, MyFoc.Ibeta);
//        EncodeValue = __HAL_TIM_GET_COUNTER(&htim3);                                     
//        encode_e_theta = Encode_get_e_theta(EncodeValue);                                
//        CurrentLoop_Encode(&MyFoc, &C_PI, IsensU, IsensV, IsensW, Iqref, encode_e_theta);
     
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


/**▲▲▲定时作用 开环斜率 编码器速度计算▲▲▲**/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)  
{
    static uint16_t value = 0;                          //编码器值暂存
    
    if (htim->Instance == TIM6)                         // TIM6计时 自增角斜率 5,000Hz 0.2ms算一次
    {
//        if(ThetaUp_Flag)  my_theta += 0.03f ;
//        my_theta = Normalize_theta(my_theta);                                                 //0-2pi
    }
    
    if (htim->Instance == TIM7)                         // TIM7计时 编码器算速度 1,000Hz 1ms算一次
    {
        value        = __HAL_TIM_GET_COUNTER(&htim3);
        encode_speed = Encode_get_speed(value, 10, &encode_cnt);
    }
    
    if (htim->Instance == TIM3)                         // TIM3编码器更新中断  进一次代表转了一圈
    {
        encode_cnt ++;
    }
    
    
}//TIM定时中断






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

/* USER CODE END 4 */

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
