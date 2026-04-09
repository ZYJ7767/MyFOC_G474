#include "Foc_Function.h"
#include "math.h"
#include "stdint.h"
#include "tim.h"
#include "arm_math.h"


FOC_TypeDef        MyFoc = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PI_CURRENT_TypeDef C_PI  = {0,0,0,0,0,0,0,0};
PI_SPEED_TypeDef   S_PI  = {0,0,0,0,0};
MTPA_TypeDef       MTPA  = {0.223,0.00044,0.00f,0.000363,0.0005325};

Resonant_Handle    PR_Id = {0, 20, 0, 0, 0, 0};
Resonant_Handle    PR_Iq = {0, 20, 0, 0, 0, 0};

/***************FOC功能函数*****************/

//限幅函数
float My_limit(float *limit, float limit_max, float limit_min)
{
    if(*limit > limit_max){*limit = limit_max;}
    if(*limit < limit_min){*limit = limit_min;}
    return *limit;
}


//归一化函数  将角度限制在0到2pi之间
float Normalize_theta(float theta)
{

    const float TwoPi = 6.283185307f;

    if (theta >= TwoPi)
    {
        theta -= TwoPi;
    }
    else if (theta < 0.0f)
    {
        theta += TwoPi;
    }
    return theta;

}


//clarke变换(等幅值)
void Clarke(FOC_TypeDef *Foc)
{
    Foc->Ialpha = Foc->Iu; // Ia
    Foc->Ibeta  = _1_sqrt3 * Foc->Iu + _2_sqrt3 * Foc->Iv;
}


//park变换
void Park(FOC_TypeDef *Foc , float theta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    arm_sin_cos_f32(theta * RAD_TO_DEG, &SinValue, &CosValue);
    
    Foc->Id =  Foc->Ialpha * CosValue + Foc->Ibeta * SinValue;
    Foc->Iq = -Foc->Ialpha * SinValue + Foc->Ibeta * CosValue;
}


//invpark变换
void Invpark(FOC_TypeDef *Foc , float theta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;

    arm_sin_cos_f32(theta * RAD_TO_DEG, &SinValue, &CosValue);

    Foc->Ualpha = Foc->Ud * CosValue - Foc->Uq * SinValue;
    Foc->Ubeta  = Foc->Ud * SinValue + Foc->Uq * CosValue;
}


//SVPWM算法
void Svpwm(FOC_TypeDef *Foc)
{
    /****************第一步扇区判断***************/ 
    //求Uref123 在这里设为u1u2u3
    
    float u1 = Foc->Ubeta;
    float u2 = _sqrt3_2  * Foc->Ualpha - Foc->Ubeta * _1_2;
    float u3 = -_sqrt3_2 * Foc->Ualpha - Foc->Ubeta * _1_2;
    
    //获取扇区号N
    
    uint8_t A =u1>0?1:0;
    uint8_t B =u2>0?1:0;
    uint8_t C =u3>0?1:0;
    uint8_t N =4*C+2*B+A;

   /****************矢量作用时间计算***************/ 
    float X =  u1 * (_sqrt3*TS/Udc);
    float Y = -u3 * (_sqrt3*TS/Udc);
    float Z = -u2 * (_sqrt3*TS/Udc);
    
    float Tm = 0;
    float Tn = 0;
    
    switch (N){
        case 1:
                Tm = Z;
                Tn = Y;
                break ;
        case 2:
                Tm = Y;
                Tn = -X;     
                break ;            
        case 3:
                Tm = -Z;
                Tn = X;      
                break ;    
        case 4:
                Tm = -X;
                Tn = Z;             
                break ;    
        case 5:
                Tm = X;
                Tn = -Y;     
                break ;    
        case 6:
                Tm = -Y;
                Tn = -Z;      
                break ;    
    }
    
    //过调制
    if((Tm+Tn)>TS){
        Tm = (Tm*TS)/(Tm+Tn);
        Tn = (Tn*TS)/(Tm+Tn);
    }   
    
    /****************三路PWM占空比计算***************/ 
    float Ta=(TS-Tm-Tn)/4;
    float Tb= Ta + Tm/2;
    float Tc= Tb + Tn/2;
    
    switch (N)
    {
        case 1:
                Foc->Tcm1=Tb;
                Foc->Tcm2=Ta;
                Foc->Tcm3=Tc;
                break ;
        case 2:
                Foc->Tcm1=Ta;
                Foc->Tcm2=Tc;
                Foc->Tcm3=Tb;
                break ;
        case 3:
                Foc->Tcm1=Ta;
                Foc->Tcm2=Tb;
                Foc->Tcm3=Tc;
                break ;
        case 4:
                Foc->Tcm1=Tc;
                Foc->Tcm2=Tb;
                Foc->Tcm3=Ta;
                break ;
        case 5:
                Foc->Tcm1=Tc;
                Foc->Tcm2=Ta;
                Foc->Tcm3=Tb;
                break ;
        case 6:
                Foc->Tcm1=Tb;
                Foc->Tcm2=Tc;
                Foc->Tcm3=Ta; 
                break ;
    }
    
    /****************三路PWM占空比输出***************/ 
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Foc->Tcm1);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Foc->Tcm2);
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Foc->Tcm3);
}

//VF开环综合函数
void VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta)
{
    Foc->Ud = Ud;
    Foc->Uq = Uq;
    Invpark(Foc,theta);
    Svpwm(Foc);
}

//IF开环综合函数       (可以作为电流环，只是角度自定义)
void IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI_ctrl, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    PI_ctrl->Iq_ref = Iq_ref;
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,PI_ctrl);
    Invpark(Foc,theta);
    Svpwm(Foc);
}


//电流环（编码器） 闭环综合函数
void CurrentLoop_Encode(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI_ctrl, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    PI_ctrl->Iq_ref = Iq_ref;
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,PI_ctrl);
    Invpark(Foc,theta);
    Svpwm(Foc);
}


//SMO滑膜观测器 电流环 综合函数
void SMO_C_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI_ctrl, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    PI_ctrl->Iq_ref = Iq_ref;
    PI_ctrl->Id_ref = 0;
    
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,PI_ctrl);
    Invpark(Foc,theta);
    Svpwm(Foc);
}

//SMO滑膜观测器 速度环电流环 双环综合函数
void SMO_S_C_Control(FOC_TypeDef *Foc,PI_SPEED_TypeDef *S_PI, PI_CURRENT_TypeDef *C_PI, float IU, float IV, float IW, float Speed_ref, float theta)
{
    static uint8_t S_cnt = 0; 
    
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;

    Clarke(Foc);
    Park(Foc,theta);
    
    float Iq_ref;
    
    S_cnt++;
    if(S_cnt > 0)
    {
        S_PI->speed_ref = Speed_ref;
        SpeedPI(Foc, S_PI, &Iq_ref);
    }
    
    C_PI->Iq_ref = Iq_ref;
    C_PI->Id_ref = 0;
    
    CurrentPI(Foc, C_PI);
    Invpark(Foc,theta);
    Svpwm(Foc);
}

/***************PID控制器功能函数*****************/

//(1)电流环PID控制器
void CurrentPI(FOC_TypeDef *Foc , PI_CURRENT_TypeDef *PI_ctrl)
{

    float ud_pi, uq_pi;
    float ud_ff, uq_ff = 0;
    float we;
    
    //1.dq轴计算偏差
    PI_ctrl->err_Id = PI_ctrl->Id_ref - Foc->Id;
    PI_ctrl->err_Iq = PI_ctrl->Iq_ref - Foc->Iq;
    
    //2.累计积分，并限幅
    PI_ctrl->Id_KI_sum += PI_ctrl->err_Id;
    PI_ctrl->Iq_KI_sum += PI_ctrl->err_Iq;
    
    if (PI_ctrl->Id_KI_sum > 600)  PI_ctrl->Id_KI_sum =  600;
    if (PI_ctrl->Id_KI_sum < -600) PI_ctrl->Id_KI_sum = -600;
    if (PI_ctrl->Iq_KI_sum > 1300)  PI_ctrl->Iq_KI_sum =  1300;
    if (PI_ctrl->Iq_KI_sum < -1300) PI_ctrl->Iq_KI_sum = -1300;
    
    //3.计算PI输出值，Ud和Uq, 并限幅
    ud_pi =(PI_ctrl->Kp * PI_ctrl->err_Id) + (PI_ctrl->Ki * PI_ctrl->Id_KI_sum);
    uq_pi =(PI_ctrl->Kp * PI_ctrl->err_Iq) + (PI_ctrl->Ki * PI_ctrl->Iq_KI_sum);
    
    we = Foc->speed * (2.0f * pi / 60.0f) * Pn;
    
    ud_ff = -we * Lq_H * PI_ctrl->Iq_ref;
    uq_ff =  we * Ld_H * PI_ctrl->Id_ref + we * PSI_F;
    
    float ud_pr = PR_Update(&PR_Id, PI_ctrl->err_Id, we, 0.0001f);
    float uq_pr = PR_Update(&PR_Iq, PI_ctrl->err_Iq, we, 0.0001f);
    
    Foc->Ud = ud_pi+ ud_ff + ud_pr;
    Foc->Uq = uq_pi+ uq_ff + uq_pr;
    
    //4.输出限幅
    if (Foc->Ud > 13)   Foc->Ud =  13;
    if (Foc->Ud < -13)  Foc->Ud = -13;
    if (Foc->Uq > 20)   Foc->Uq =  20;
    if (Foc->Uq < -20)  Foc->Uq = -20;
}


//(2)速度环PID控制器
void SpeedPI(FOC_TypeDef *Foc, PI_SPEED_TypeDef *PI_ctrl, float *Iqref)
{
    float Iq_final;
    
    //1.dq轴计算偏差
    PI_ctrl->err_speed = PI_ctrl->speed_ref - Foc->speed;
    
    //2.累计积分，并限幅
    PI_ctrl->speed_KI_sum += PI_ctrl->err_speed;
    
    if (PI_ctrl->speed_KI_sum >  40000.0f) PI_ctrl->speed_KI_sum =  40000.0f;
    if (PI_ctrl->speed_KI_sum < -40000.0f) PI_ctrl->speed_KI_sum = -40000.0f;
    
    //3.计算PI输出值Iqref，作为电流环输入
    Iq_final=(PI_ctrl->Kp * PI_ctrl->err_speed) + (PI_ctrl->Ki * PI_ctrl->speed_KI_sum);

    //4.输出限幅
    if (Iq_final > 23.0f)  Iq_final =  23.0f;
    if (Iq_final < 0.0f)  Iq_final = 0.0f;
    
    (*Iqref) = Iq_final;
}


/***************其他控制策略函数*****************/

// 准谐振控制器 (PR) 核心差分方程
float PR_Update(Resonant_Handle *pr, float x_in, float We_rad, float Ts)
{
    // 1. 锁定目标消除频率：6 倍的电角速度
    float w0 = 6.0f * fabsf(We_rad); 
    
    // 如果电频率太低或速度接近 0，不需要谐振补偿，直接返回0
    if(w0 < 15.0f || pr->Kr < 0.1f) {
        return 0.0f; 
    }

    // 2. Tustin 双线性变换离散化参数计算
    float w0_sq = w0 * w0;
    float Ts_sq_inv = 1.0f / (Ts * Ts);
    float Ts_inv = 1.0f / Ts;

    // 公共分母 temp
    float temp = 4.0f * Ts_sq_inv + 4.0f * pr->Wc * Ts_inv + w0_sq;
    
    // 差分方程系数 b0, b1, b2, a1, a2
    float b0 =  (4.0f * pr->Kr * pr->Wc * Ts_inv) / temp;
    float b1 =  0.0f;
    float b2 = -(4.0f * pr->Kr * pr->Wc * Ts_inv) / temp;
    
    float a1 =  (2.0f * w0_sq - 8.0f * Ts_sq_inv) / temp;
    float a2 =  (4.0f * Ts_sq_inv - 4.0f * pr->Wc * Ts_inv + w0_sq) / temp;

    // 3. 计算输出
    float y_out = b0 * x_in + b1 * pr->x_prev1 + b2 * pr->x_prev2 
                  - a1 * pr->y_prev1 - a2 * pr->y_prev2;

    // 4. 更新历史状态
    pr->x_prev2 = pr->x_prev1;
    pr->x_prev1 = x_in;
    pr->y_prev2 = pr->y_prev1;
    pr->y_prev1 = y_out;
    
    if(y_out >  3.0f) y_out =  3.0f;
    if(y_out < -3.0f) y_out = -3.0f;

    return y_out;
}


//MTPA控制函数
void MTPA_Calculate(MTPA_TypeDef *MTPA , PI_CURRENT_TypeDef *PI_ctrl)
{
    //后续假如查表
}




















