#include "encoder.h"

//▲编码器获取电角度
uint16_t Encode_get_e_theta(uint16_t value)
{
    uint16_t e_theta;
    e_theta = Pn * value * 2 * pi / 4000;
    return e_theta;
}

//▲编码器算速度                                                                (value:当前编码器值  ms：多少毫秒算一次)
float Encode_get_speed(uint32_t value, uint8_t ms, uint16_t *encode_cnt)
{
    uint8_t i = 0, j = 0;                                                       /* 冒泡法序列 */
    float temp = 0.0;                                                           /* 冒泡法缓存区 */
    
    static uint8_t  ms_count = 0, k = 0;                                        /* 用于对计算速度间隔事件计时 k用于数组解引用*/
    static float speed_arr[10] = {0.0};                                         /* 存储10次速度，进行滤波运算 */
    
    static uint32_t encode_now   = 0;                                           /* 当前值 */
    static uint32_t encode_old   = 0;                                           /* 上次值 */
    static uint32_t encode_delta = 0;                                           /*  差值  */
    
    static float speed =0;                                                      /* 最后算出得速度值*/
    
    if (ms_count == ms)
    {
        if((*encode_cnt) > 0) 
        {
            value += 4000 * (*encode_cnt);
            *encode_cnt = 0;
        }
        encode_now     = value;                                                 /* 取出编码器当前计数值 */
        encode_delta   = encode_now - encode_old;                               /* 计算编码器计数值的变化量 */
        speed_arr[k++] = (float)(encode_delta * ((4000 / ms) * 60.0) / 4000);
        encode_old     = encode_now;
        
        if (k == 10)                                                            /* 10个值，排序，去掉最大最小值后滤波*/
        {
            for (i = 10; i >= 1; i--)                                           /* 冒泡排序*/
            {
                for (j = 0; j < (i - 1); j++) 
                {
                    if (speed_arr[j] > speed_arr[j + 1])                        /* 数值比较 */
                    { 
                        temp = speed_arr[j];                                    /* 数值换位 */
                        speed_arr[j] = speed_arr[j + 1];
                        speed_arr[j + 1] = temp;
                    }
                }
            }
            temp = 0.0;
            for (i = 2; i < 8; i++)                                             /* 去除两边高低数据 */
            {
                temp += speed_arr[i];                                           /* 将中间数值累加 */
            }
            temp = (float)(temp / 6);                                           /* 求速度平均值 */
//            speed = LowPassFilter(temp , 0.48);                                 /* 一阶滤波 */
            k = 0;
        }
        ms_count = 0;
    }
    ms_count ++;

    return speed;

}






