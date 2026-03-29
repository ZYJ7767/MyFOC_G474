#ifndef __OBSERVER_H
#define __OBSERVER_H

#include "Foc_Function.h"
#include "stdint.h"
#include "math.h"
#include "main.h"


//电机基波模型： Eα = Uα - Rs*Iα - Ls*diα/dt；β同理；

/******* 电机参数结构体 *******/
typedef struct
{
    float Rs ;
    float Ls ;
    float phi_f;
    float Ld ;
    float Lq ;
    
}StepMotor;


/******* SMO 结构体 *******/
typedef struct
{
    float A;                // A = exp(-R/LsTs)
    float B;                // B = (1-A)1/R    袁雷
    float K;                // 滑膜增益
    float Ts;               // 采样周期
    
    float est_Theta;        // 转子位置
    float prev_theta;       // 上一次的转子位置
    
    float est_Speed;        // 转子速度
    
    float est_ialpha;       // 估计的 i_alpha 电流
    float est_ibeta;        // 估计的 i_beta 电流
    
    float est_ialpha_dt;
    float est_ibeta_dt;
    
    float phase_delay;      // 相位延迟值ֵ
    
    float E_alpha;          // alpha轴反电动势
    float E_beta;           // beta轴反电动势
    
    
} SlidingModeObserver;


/******* PLL结构体 *******/
typedef struct
{
    float Kp ;              //2ζωn         当ζ=0.707时，自然频率ωn就为系统带宽ωb
    float Ki ;              //ωn*ωn
    float Up ;
    float Ui ;
    
    float Ts ;
    float Err ;
    
    float Est_we;           //估计电角速度
    float Est_RPM;       //估计转速
    
    float Est_theta;        //估计电角度
    uint16_t Est_theta_int; //1024格式的电角度
    float Pre_Est_Theta;    //上一次估计电角度
    
}PLL_Handle;




/**** 结构体声明 ****/
extern StepMotor Mo;
extern SlidingModeObserver SMO;
extern PLL_Handle PLL;

/**** 函数声明 ****/
uint16_t BEF_calculate(FOC_TypeDef *Foc,StepMotor *SteoMotor);

float sign(float x);                                                                                                            //符号函数
float sat(float x);
float sigmoid(float x);
void  PLL_calculate(PLL_Handle *PLL ,float Ealpha ,float Ebeta);                                                                //PLL函数

float SMO_Update(SlidingModeObserver *smo, float u_alpha, float u_beta, float i_alpha, float i_beta);                           //SMO atan
float SMO_PLL_Update(SlidingModeObserver *smo, PLL_Handle *PLL, float u_alpha, float u_beta, float i_alpha, float i_beta);      //SMO PLL


/******* IF->SMO *******/
typedef enum {
    BLEND_STATE_IF_ONLY    = 0,   // 只IF驱动
    BLEND_STATE_CONVERGING = 1,   // 判断SMO是否收敛
    BLEND_STATE_BLENDING   = 2,   // 融合阶段IF+SMO
    BLEND_STATE_SMO_ONLY   = 3    // 只SMO驱动
} BlendState_t;

typedef struct {
    float        threshold;          // 容许误差rad
    uint16_t     hold_cnt_max;       // 在容许误差保持的计数
    uint16_t     openloop_cnt_min;   // IF min cycles
    uint16_t     blend_steps;        // 融合段细分步数
    uint16_t     hold_cnt;
    uint16_t     blend_cnt;
    BlendState_t state;              //当前阶段
    float        alpha;              //权重 0=IF only, 1=SMO only 
    float        final_theta;
} ThetaBlend_t;

extern ThetaBlend_t Blend;

float IF_SMO_Blend(ThetaBlend_t *blend, float if_theta, float smo_theta, uint16_t openloop_cnt);
float SMO_GetPhaseComp(float mech_rpm);  //相位补偿

#endif

