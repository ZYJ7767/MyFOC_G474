#include "Observer.h"
#include "Filter.h"
#include "arm_math.h"


LPF1_t g_smo_ealpha_lpf = {0};
LPF1_t g_smo_ebeta_lpf  = {0};

StepMotor Mo            = {0.1175, 0.000181, 0.00f, 0.00f, 0.00f};                           //电阻Rs=0.223025 电感Ls = 0.000444463H
SlidingModeObserver SMO = {0.93714, 0.5349, 5.0, 0.0001, 0, 0, 0, 0, 0, 0, 0, 0};              //k=0.75-6.25,Ts=0.0001s 
PLL_Handle PLL          = { 888.4f, 39.49f, 0, 0, 0.0001, 0, 0, 0, 0, 0};                        //Kp=2ζωn=2*0.707*7     Ki = ωn*ωn=7^2

ThetaBlend_t Blend      = {0.85f, 100, 20000, 500, 0, 0, BLEND_STATE_IF_ONLY, 0.0f, 0.0f};     //threshold=0.3f rad (~17deg), hold_cnt_max=500, openloop_cnt_min=10000, blend_steps=500




/**************** 反电动势法 （数学法） 基波模型 ****************/
uint16_t BEF_calculate(FOC_TypeDef *Foc,StepMotor *SteoMotor)
{
    Foc->PIalpha     = Foc->Ialpha - Foc->Ialpha_prev;
    Foc->Ialpha_prev = Foc->Ialpha;
    Foc->PIbeta      = Foc->Ibeta  - Foc->Ibeta_prev;
    Foc->Ibeta_prev  = Foc->Ibeta;
    
    Foc->Ealpha      = Foc->Ualpha - (SteoMotor->Rs * Foc->Ialpha) - (SteoMotor->Ls * Foc->PIalpha);
    Foc->Ebeta       = Foc->Ubeta  - (SteoMotor->Rs * Foc->Ibeta)  - (SteoMotor->Ls * Foc->PIbeta);
    
    float theta = atan2f((-Foc->Ealpha),Foc->Ebeta);
    
    if(theta < 0) theta += 2 * 3.141592;                    //角度归到0-2pi

    return theta;
}

/**************** SMO+反正切 滑膜观测器 袁雷离散 ****************/
/***** 符号函数 *****/
float sign(float x)
{
    if (x > 0) return 1.0f;
    if (x < 0) return -1.0f;
    return 0.0f;
}

float sat(float x)
{
    if (x > 0.5f)  return 1.0f;
    if (x < -0.5f) return -1.0f;
    return x / 0.5f;   // 边界层内线性
}

float sigmoid(float x)
{
    return tanhf(3.0f * x);
}

/*****更新滑膜观测器 *****/
float SMO_Update(SlidingModeObserver *smo,float u_alpha, float u_beta, float i_alpha, float i_beta) 
{
    // 更新alpha轴
    smo->E_alpha = smo->K * sign(smo->est_ialpha - i_alpha);
    smo->E_alpha = LowPassFilter(smo->E_alpha ,0.65);
    smo->est_ialpha = smo->A * smo->est_ialpha + smo->B * (u_alpha - smo->E_alpha);

    // 更新beta轴
    smo->E_beta  = smo->K * sign(smo->est_ibeta - i_beta);
    smo->E_beta  = LowPassFilter(smo->E_beta  ,0.65);
    smo->est_ibeta = smo->A * smo->est_ibeta + smo->B * (u_beta - smo->E_beta);
    
    // 直接反正切
    float e_theta = atan2f((-smo->E_alpha),smo->E_beta) ;
    if(e_theta < 0) e_theta += 2 * 3.141592;
    
    
    return e_theta;
}

/**************** SMO+PLL 滑膜观测器 袁雷离散 ****************/
float SMO_PLL_Update(SlidingModeObserver *smo, PLL_Handle *PLL, float u_alpha, float u_beta, float i_alpha, float i_beta) 
{
    // 更新alpha轴
    smo->E_alpha = smo->K * sat(smo->est_ialpha - i_alpha);
    smo->est_ialpha = smo->A * smo->est_ialpha + smo->B * (u_alpha - smo->E_alpha);    
    smo->E_alpha = LPF1_Update(&g_smo_ealpha_lpf, smo->E_alpha, 0.40f);//LowPassFilter(smo->E_alpha ,0.65);
    


    // 更新beta轴
    smo->E_beta  = smo->K * sat(smo->est_ibeta - i_beta);
    smo->est_ibeta = smo->A * smo->est_ibeta + smo->B * (u_beta - smo->E_beta);    
    smo->E_beta  = LPF1_Update(&g_smo_ebeta_lpf , smo->E_beta , 0.40f);//LowPassFilter(smo->E_beta  ,0.65);

    
    // PLL锁定角度
    PLL_calculate(PLL, smo->E_alpha, smo->E_beta );
    
    return PLL->Est_theta;
}


/**************** PLL锁相环计算函数 ****************/ 
void PLL_calculate(PLL_Handle *PLL ,float Ealpha ,float Ebeta)
{
    float SinValue = 0.0f;
    float CosValue = 0.0f;
    float Em_Mag   = 0.0f;  // [新增] 用于存储反电动势幅值
    
    arm_sin_cos_f32(PLL->Est_theta * RAD_TO_DEG, &SinValue, &CosValue);
    
    PLL->Err = -Ealpha *CosValue - Ebeta *SinValue;
    Em_Mag = sqrtf(Ealpha * Ealpha + Ebeta * Ebeta);
    PLL->Err = PLL->Err / (Em_Mag + 0.001f);
    
    PLL->Err = (PLL->Err > 0.5236f)  ?  (0.5236f) : (PLL->Err);                   //当Δθ小于pi/6时，认为sin（Δθ）= Δθ
    PLL->Err = (PLL->Err < -0.5236f) ? (-0.5236f) : (PLL->Err);

    PLL->Up  = PLL->Err * PLL->Kp;
    PLL->Ui += PLL->Err * PLL->Ki;
    
    if (PLL->Ui > 2500.0f)  PLL->Ui = 2500.0f;
    if (PLL->Ui < -2500.0f) PLL->Ui = -2500.0f;
    
    PLL->Est_we  = PLL->Up + PLL->Ui;
    
    if (PLL->Est_we > 2500.0f)  PLL->Est_we = 2500.0f;
    if (PLL->Est_we < -2500.0f) PLL->Est_we = -2500.0f;
    

    PLL->Est_RPM = PLL->Est_we * 2.387f;                                //1.194f   8对极   2.387f   4duiji
//    MyFoc.speed = PLL->Est_RPM;                                                   //及时将观测速度传给foc结构体
    
    PLL->Est_theta += PLL->Est_we * PLL->Ts;
    
    PLL->Est_theta = Normalize_theta(PLL->Est_theta);
}



/**************** IF/SMO 切换判断&加权切换函数 ****************/
float IF_SMO_Blend(ThetaBlend_t *blend, float if_theta, float smo_theta, uint16_t openloop_cnt)
{
    /* diff = smo - if, wrapped to (-pi, pi] via shortest arc.
     * Equivalent to: final = IF*(1-alpha) + alpha*SMO
     * but handles 0/2pi boundary correctly.
     * e.g. IF=0.1, SMO=6.18 -> diff=-0.1 (not +6.08), so blend stays near 0.
     */
    float diff = fmodf(smo_theta - if_theta + 3.0f * pi, 2.0f * pi) - pi;  //环形角度 最短弧处理 

    switch(blend->state)
    {
        case BLEND_STATE_IF_ONLY:
            blend->final_theta = if_theta;
            /* Wait until IF has run long enough for SMO to converge */
            if(openloop_cnt >= blend->openloop_cnt_min)
                blend->state = BLEND_STATE_CONVERGING;
            break;

        case BLEND_STATE_CONVERGING:
            blend->final_theta = if_theta;
            /* Check if observer angle tracks IF angle within threshold */
            if(fabsf(diff) < blend->threshold)
            {
                blend->hold_cnt++;
                /* Must remain close for hold_cnt_max consecutive cycles */
                if(blend->hold_cnt >= blend->hold_cnt_max)
                {
                    blend->hold_cnt = 0;
                    blend->state    = BLEND_STATE_BLENDING;
                }
            }
            else
            {
                blend->hold_cnt = 0;  /* reset on any divergence, prevent false trigger */
            }
            break;

        case BLEND_STATE_BLENDING:
            /* Linearly ramp alpha 0->1 over blend_steps cycles */
            blend->blend_cnt++;
            blend->alpha = (float)blend->blend_cnt / (float)blend->blend_steps;
            if(blend->alpha >= 1.0f)
            {
                blend->alpha     = 1.0f;
                blend->blend_cnt = blend->blend_steps;
                blend->state     = BLEND_STATE_SMO_ONLY;
            }
            /* final = IF*(1-alpha) + alpha*SMO, using diff for shortest-arc interpolation */
            blend->final_theta = Normalize_theta(if_theta + blend->alpha * diff);
            break;

        case BLEND_STATE_SMO_ONLY:
            blend->final_theta = smo_theta;
            break;
    }

    return blend->final_theta;
}


/**************** 相位补偿 ****************/
float SMO_GetPhaseComp(float mech_rpm)
{
    // 总相位补偿 = 电角速度 * 等效延时（SMO + LPF + 计算延迟）
    // 先给一个保守值，后续可微调
    const float Td_eq_s  = 180e-6f;   // 建议起始 140e-6 ~ 260e-6
    const float rpm_ramp = 500.0f;    // 低速渐入，避免低速抖
    const float comp_max = 0.35f;     // 最大约20度(0.35rad)
    const float lpf_a    = 0.05f;     // 补偿量平滑，防止抖动

    float sign = (mech_rpm >= 0.0f) ? 1.0f : -1.0f;
    float rpm  = fabsf(mech_rpm);

    // 电角速度(rad/s): we = wm * pole_pairs
    float we = rpm * (2.0f * pi / 60.0f) * Pn;

    // 线性相位超前（弧度）
    float comp = we * Td_eq_s;

    // 低速按比例渐入
    if (rpm < rpm_ramp) {
        comp *= (rpm / rpm_ramp);
    }

    // 限幅
    if (comp > comp_max) comp = comp_max;
    if (comp < 0.0f)     comp = 0.0f;

    // 一阶平滑
    static float comp_f = 0.0f;
    comp_f += lpf_a * (comp - comp_f);

    return sign * comp_f;
}





