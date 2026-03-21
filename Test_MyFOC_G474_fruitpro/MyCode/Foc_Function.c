#include "Foc_Function.h"
#include "math.h"
#include "stdint.h"
#include "tim.h"


FOC_TypeDef        MyFoc = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
PI_CURRENT_TypeDef C_PI  = {0,0,0,0,0,0,0,0};
PI_SPEED_TypeDef   S_PI  = {0,0,0,0,0};
MTPA_TypeDef       MTPA  = {0.223,0.00044,0.00f,0.000363,0.0005325};

/***************▲▲FOC功能函数▲▲*****************/

//▲限幅函数
float My_limit(float *limit, float limit_max, float limit_min)
{
    if(*limit > limit_max){*limit = limit_max;}
    if(*limit < limit_min){*limit = limit_min;}
    return *limit;
}


//▲归一化函数  将角度限制在0到2pi之间
float Normalize_theta(float theta)
{
    float a;
    a = fmodf(theta,2*pi);   //取余运算
    
    return a>0?a:(a+2*pi);
}


//▲clarke变换(等幅值)
void Clarke(FOC_TypeDef *Foc)
{
    Foc->Ialpha = Foc->Iu; // Ia
    Foc->Ibeta  = _1_sqrt3 * Foc->Iu + _2_sqrt3 * Foc->Iv;
}


//▲park变换
void Park(FOC_TypeDef *Foc , float theta)
{
    Foc->Id =  Foc->Ialpha * cosf(theta) + Foc->Ibeta * sinf(theta);
    Foc->Iq = -Foc->Ialpha * sinf(theta) + Foc->Ibeta * cosf(theta);
}


//▲invpark变换
void Invpark(FOC_TypeDef *Foc , float theta)
{
    Foc->Ualpha = Foc->Ud * cosf(theta) - Foc->Uq * sinf(theta);
    Foc->Ubeta  = Foc->Ud * sinf(theta) + Foc->Uq * cosf(theta);
}


//▲SVPWM算法
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

//▲VF开环综合函数
void VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta)
{
    Foc->Ud = Ud;
    Foc->Uq = Uq;
    Invpark(Foc,theta);
    Svpwm(Foc);
}

//▲IF开环综合函数       (也是电流环，只是角度自定义)
void IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    PI->Iq_ref = Iq_ref;
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,PI);
    Invpark(Foc,theta);
    Svpwm(Foc);
}


//▲电流环（编码器） 闭环综合函数
void CurrentLoop_Encode(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    PI->Iq_ref = Iq_ref;
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,PI);
    Invpark(Foc,theta);
    Svpwm(Foc);
}


//▲SMO滑膜观测器 电流环 综合函数
void SMO_C_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI, float IU, float IV, float IW, float Iq_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;
    PI->Iq_ref = Iq_ref;
    PI->Id_ref = 0;
    
    
    Clarke(Foc);
    Park(Foc,theta);
    CurrentPI(Foc,PI);
    Invpark(Foc,theta);
    Svpwm(Foc);
}

//▲SMO滑膜观测器 速度环电流环 双环综合函数
void SMO_S_C_Control(FOC_TypeDef *Foc,PI_SPEED_TypeDef *S_PI, PI_CURRENT_TypeDef *C_PI, float IU, float IV, float IW, float Speed_ref, float theta)
{
    Foc->Iu = IU;
    Foc->Iv = IV;
    Foc->Iw = IW;

    Clarke(Foc);
    Park(Foc,theta);
    
    float Iq_ref;
    
    S_PI->speed_ref = Speed_ref;
    SpeedPI(Foc, S_PI, &Iq_ref);
    
    C_PI->Iq_ref = Iq_ref;
    C_PI->Id_ref = 0;
    
    CurrentPI(Foc, C_PI);
    Invpark(Foc,theta);
    Svpwm(Foc);
}

/***************▲▲PID控制器功能函数▲▲*****************/

//(1)▲电流环PID控制器
void CurrentPI(FOC_TypeDef *Foc , PI_CURRENT_TypeDef *PI)
{

    float ud_pi, uq_pi;
//    float ud_ff, uq_ff = 0;
//    float we;
    
    //1.dq轴计算偏差
    PI->err_Id = PI->Id_ref - Foc->Id;
    PI->err_Iq = PI->Iq_ref - Foc->Iq;
    
    //2.累计积分，并限幅
    PI->Id_KI_sum += PI->err_Id;
    PI->Iq_KI_sum += PI->err_Iq;
    
    if (PI->Id_KI_sum > 430)  PI->Id_KI_sum =  430;
    if (PI->Id_KI_sum < -430) PI->Id_KI_sum = -430;
    if (PI->Iq_KI_sum > 1000)  PI->Iq_KI_sum =  1000;
    if (PI->Iq_KI_sum < -1000) PI->Iq_KI_sum = -1000;
    
    //3.计算PI输出值，Ud和Uq, 并限幅
    ud_pi =(PI->Kp * PI->err_Id) + (PI->Ki * PI->Id_KI_sum);
    uq_pi =(PI->Kp * PI->err_Iq) + (PI->Ki * PI->Iq_KI_sum);
    
//    we = Foc->speed * (2.0f * pi / 60.0f) * Pn;
//    ud_ff = -we * Lq_H * Foc->Iq;
//    uq_ff =  we * Ld_H * Foc->Id + we * PSI_F;
    
    Foc->Ud = ud_pi; //+ ud_ff;
    Foc->Uq = uq_pi; //+ uq_ff;
    
    //4.输出限幅
    if (Foc->Ud > 13)   Foc->Ud =  13;
    if (Foc->Ud < -13)  Foc->Ud = -13;
    if (Foc->Uq > 13)   Foc->Uq =  13;
    if (Foc->Uq < -13)  Foc->Uq = -13;
}


//(2)▲速度环PID控制器
void SpeedPI(FOC_TypeDef *Foc, PI_SPEED_TypeDef *PI, float *Iqref)
{
    float Iq_final;
    
    //1.dq轴计算偏差
    PI->err_speed = PI->speed_ref - Foc->speed;
    
    //2.累计积分，并限幅
    PI->speed_KI_sum += PI->err_speed;
    
    if (PI->speed_KI_sum >  60000.0f) PI->speed_KI_sum =  60000.0f;
    if (PI->speed_KI_sum < -60000.0f) PI->speed_KI_sum = -60000.0f;
    
    //3.计算PI输出值Iqref，作为电流环输入
    Iq_final=(PI->Kp * PI->err_speed) + (PI->Ki * PI->speed_KI_sum);

    //4.输出限幅
    if (Iq_final > 12.0f)  Iq_final =  12.0f;
    if (Iq_final < -1.0f)  Iq_final = -1.0f;
    
    (*Iqref) = Iq_final;
}


/***************▲▲其他控制策略函数▲▲*****************/
//▲MTPA控制函数
void MTPA_Calculate(MTPA_TypeDef *MTPA , PI_CURRENT_TypeDef *PI)
{
    float delta_L = MTPA->Ld - MTPA->Lq;
    
    if(delta_L <=0.0001f|| delta_L >=-0.0001f)
    {
        
    }
    
}




















