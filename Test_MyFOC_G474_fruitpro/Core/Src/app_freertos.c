/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adc.h"
#include "Led_Key.h"
#include "dwt_timer.h"
#include "LCD_SPI.h"
#include <stdio.h>
#include "Foc_Function.h" 
#include "BDC_Ctrl.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern float Vbus;
extern uint16_t ADC_Vbus;
extern uint8_t key;
extern DWT_Time_t t ;
extern float Speedref;
extern FOC_TypeDef MyFoc;

uint16_t UI_Buf[144 * 32];
uint16_t cnt = 0;

volatile uint8_t Target_flg = 0;   // 鐘舵?佹爣蹇楋細0琛ㄧず鍋滄锛?1琛ㄧず姝ｈ浆锛?2琛ㄧず鍙嶈浆
volatile uint16_t BDC_Speed = 10000;
volatile uint8_t Limit;

/* USER CODE END Variables */
/* Definitions for Task_10ms */
osThreadId_t Task_10msHandle;
uint32_t Task_10msBuffer[ 256 ];
osStaticThreadDef_t Task_10msControlBlock;
const osThreadAttr_t Task_10ms_attributes = {
  .name = "Task_10ms",
  .stack_mem = &Task_10msBuffer[0],
  .stack_size = sizeof(Task_10msBuffer),
  .cb_mem = &Task_10msControlBlock,
  .cb_size = sizeof(Task_10msControlBlock),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Task_1ms */
osThreadId_t Task_1msHandle;
uint32_t Task_1msBuffer[ 256 ];
osStaticThreadDef_t Task_1msControlBlock;
const osThreadAttr_t Task_1ms_attributes = {
  .name = "Task_1ms",
  .stack_mem = &Task_1msBuffer[0],
  .stack_size = sizeof(Task_1msBuffer),
  .cb_mem = &Task_1msControlBlock,
  .cb_size = sizeof(Task_1msControlBlock),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for Task_500ms */
osThreadId_t Task_500msHandle;
uint32_t Task_500msBuffer[ 512 ];
osStaticThreadDef_t Task_500msControlBlock;
const osThreadAttr_t Task_500ms_attributes = {
  .name = "Task_500ms",
  .stack_mem = &Task_500msBuffer[0],
  .stack_size = sizeof(Task_500msBuffer),
  .cb_mem = &Task_500msControlBlock,
  .cb_size = sizeof(Task_500msControlBlock),
  .priority = (osPriority_t) osPriorityLow,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartTask_10ms(void *argument);
void StartTask_1ms(void *argument);
void StartTask_500ms(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of Task_10ms */
  Task_10msHandle = osThreadNew(StartTask_10ms, NULL, &Task_10ms_attributes);

  /* creation of Task_1ms */
  Task_1msHandle = osThreadNew(StartTask_1ms, NULL, &Task_1ms_attributes);

  /* creation of Task_500ms */
  Task_500msHandle = osThreadNew(StartTask_500ms, NULL, &Task_500ms_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartTask_10ms */
/**
  * @brief  Function implementing the Task_10ms thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartTask_10ms */
void StartTask_10ms(void *argument)
{
  /* USER CODE BEGIN StartTask_10ms */
  /* Infinite loop */
  uint32_t PreviousWakeTime = osKernelGetTickCount(); 
  const uint32_t TimeIncrement = 10; 
    
  BDC_Init();               //直流电机初始化 
  uint8_t last_state = 255; //进给电机初始状态设为不同
    
  for(;;)
  {
    /***** MainTask *****/

    if(Target_flg != last_state)
    {
          if(Target_flg == 0)
          {
              BDC_Stop();
          }
          else if(Target_flg == 1)
          {
              BDC_Down_drive();
          }
          else if(Target_flg == 2)
          {
              BDC_Up_drive();
          }
          last_state = Target_flg;
    }


    if (Target_flg != 0) 
    {
          BDC_SetSpeed(BDC_Speed);
    }

    /***** Vbus ADC  *****/
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 5); 
    ADC_Vbus   = HAL_ADC_GetValue(&hadc1);
    Vbus = (float)ADC_Vbus * 0.0176757477f;

    /****** KEY_LED  ******/
    key = key_scan(0);
    if (key)
    {
      switch (key)
      {
            case KEY0_PRES:
//                LED0_TOGGLE();
                Speedref += 100;
                if(Speedref == 100 ) Speedref = 200 ;
                if(Speedref >= 2000) Speedref = 2000;
                break;

            case KEY1_PRES:
//                LED1_TOGGLE();
                Speedref -= 100;
                if(Speedref < 200 ) Speedref =  0;
                break;

            case KEY2_PRES:
//                LED0_TOGGLE();
//                LED1_TOGGLE();
               Speedref = 0.0f;
                break;
            default : break;
      }
    }

    PreviousWakeTime += TimeIncrement;
    osDelayUntil(PreviousWakeTime);

  }
  /* USER CODE END StartTask_10ms */
}

/* USER CODE BEGIN Header_StartTask_1ms */
/**
* @brief Function implementing the Task_1ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_1ms */
void StartTask_1ms(void *argument)
{
  /* USER CODE BEGIN StartTask_1ms */
    
  uint32_t PreviousWakeTime = osKernelGetTickCount(); 
  const uint32_t TimeIncrement = 1; 
    
  /* Infinite loop */
  for(;;)
  {
    Limit = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12);
    if (HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_12) == GPIO_PIN_SET) 
    {
            if (Target_flg == 2) // 假设 Target_flg == 2 是朝这个传感器的方向运动
            {
                Target_flg = 0;  // 强制停车，防止撞坏
            }
    }
    
    PreviousWakeTime += TimeIncrement;
    osDelayUntil(PreviousWakeTime);
  }
  /* USER CODE END StartTask_1ms */
}

/* USER CODE BEGIN Header_StartTask_500ms */
/**
* @brief Function implementing the Task_500ms thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask_500ms */
void StartTask_500ms(void *argument)
{
  /* USER CODE BEGIN StartTask_500ms */

    LCD_Init();
    LCD_Clear(LCD_BLACK);
    
    LCD_Draw_String_To_Buffer_X2("Ref:", UI_Buf, 144, 32, LCD_WHITE, LCD_BLACK);
    LCD_Fill_DMA(10, 20, 10 + 144 - 1, 20 + 32 - 1, UI_Buf);

    LCD_Draw_String_To_Buffer_X2("RPM:", UI_Buf, 96, 32, LCD_WHITE, LCD_BLACK);
    LCD_Fill_DMA(10, 70, 10 + 96 - 1, 70 + 32 - 1, UI_Buf);

    LCD_Draw_String_To_Buffer_X2("Iq:", UI_Buf, 48, 32, LCD_WHITE, LCD_BLACK);
    LCD_Fill_DMA(10, 120, 10 + 48 - 1, 120 + 32 - 1, UI_Buf);

    LCD_Draw_String_To_Buffer_X2("Id:", UI_Buf, 48, 32, LCD_WHITE, LCD_BLACK);
    LCD_Fill_DMA(10, 170, 10 + 48 - 1, 170 + 32 - 1, UI_Buf);

  uint32_t tick = osKernelGetTickCount();
  char str_buf[15];
    
  /* Infinite loop */
  for(;;)
  {
        //DWT_Timer_Start(&t);
        // Speedref
        sprintf(str_buf, "%8.1f", Speedref);
        LCD_Draw_String_To_Buffer_X2(str_buf, UI_Buf, 128, 32, LCD_WHITE, LCD_BLACK);
        LCD_Fill_DMA(90, 20, 90 + 128 - 1, 20 + 32 - 1, UI_Buf);

        // speed
        sprintf(str_buf, "%8.1f", MyFoc.speed);
        LCD_Draw_String_To_Buffer_X2(str_buf, UI_Buf, 128, 32, LCD_GREEN, LCD_BLACK);
        LCD_Fill_DMA(90, 70, 90 + 128 - 1, 70 + 32 - 1, UI_Buf);

        // Iq
        sprintf(str_buf, "%8.2f", MyFoc.Iq);
        LCD_Draw_String_To_Buffer_X2(str_buf, UI_Buf, 128, 32, LCD_YELLOW, LCD_BLACK);
        LCD_Fill_DMA(90, 120, 90 + 128 - 1, 120 + 32 - 1, UI_Buf);

        // Id
        sprintf(str_buf, "%8.2f", MyFoc.Id);
        LCD_Draw_String_To_Buffer_X2(str_buf, UI_Buf, 128, 32, LCD_BLUE, LCD_BLACK);
        LCD_Fill_DMA(90, 170, 90 + 128 - 1, 170 + 32 - 1, UI_Buf);
      
        //DWT_Timer_Stop(&t);
        
        tick += 500;
        osDelayUntil(tick);
      
  }
  /* USER CODE END StartTask_500ms */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

