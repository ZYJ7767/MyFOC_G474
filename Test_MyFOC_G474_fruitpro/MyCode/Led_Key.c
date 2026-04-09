#include "Led_Key.h"

/*******▲▲KEY 扫描功能函数▲▲*******/
/* @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): KEY0 > KEY1 > KEY2!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 *              KEY2_PRES, 3, KEY2按下
*/
uint8_t key_scan(uint8_t mode)
{
    static uint8_t key_up = 1;       /* 按键按松开标志 */
    static uint8_t debounce_cnt = 0; /* 任务周期消抖计数器 */
    uint8_t keyval = 0;

    /* 检查是否有键按下 */
    if (KEY0 == 0 || KEY1 == 0 || KEY2 == 0) 
    {
        if (key_up) 
        {
            debounce_cnt++;
            if (debounce_cnt >= 2)  /* 相当于 > 10ms 的消抖 */
            {
                key_up = 0;
                debounce_cnt = 0;

                if (KEY0 == 0)  keyval = KEY0_PRES;
                else if (KEY1 == 0)  keyval = KEY1_PRES;
                else if (KEY2 == 0)  keyval = KEY2_PRES;
            }
        }
    }
    else  
    {
        /* 没有任何按键按下，清空计数器，恢复松开状态 */
        debounce_cnt = 0;
        key_up = 1;
    }

    return keyval; /* 返回键值 */
}

















