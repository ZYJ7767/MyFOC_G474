#ifndef __FOC_H
#define __FOC_H


//▲电机参数
    //硬石科技无刷电机      Rs=0.223025Ω   Ls=0.000444463H    （Lq=1.06543/2mH Ld=0.7124/2mH）
    //直流无刷割草机        Rs=*****Ω     Ls=

//▲宏定义
#define  pi             3.1415926535f
#define _2sqrt3_3       1.1547005383f
#define _sqrt3_3        0.5773502691f
#define _sqrt3_2        0.8660254037844f
#define _sqrt3          1.7320508075688f
#define _1_sqrt3        0.57735026919f 
#define _2_sqrt3        1.15470053838f 
#define _1_2            0.5f
#define _2_3            0.6666666666666f


#define TS              3500     //定时器的ARR值
#define Udc             24
#define Pn              8        //极对数
#define Ld_H            0.000155f   // 先用你现有值
#define Lq_H            0.000206f   // 先用同值，不确定时先这样
#define PSI_F           0.007f      // Wb


//▲结构体
//▲FOC控制主结构体
typedef struct
{
    float Iu;
    float Iv;
    float Iw;
    
    float Ialpha;
    float Ibeta;
    
    float Id;
    float Iq;
    
    float Ud;
    float Uq;

    float Ualpha;
    float Ubeta;
    
    float Tcm1;
    float Tcm2;
    float Tcm3;
    
    /**** 无感控制 ****/
    float Ealpha;
    float Ebeta;
    
    float Ialpha_prev;
    float Ibeta_prev;
    
    float PIalpha;          //微分算子P
    float PIbeta;
    
    float speed;
    
}FOC_TypeDef;

//▲电流PI控制器结构体
typedef struct
{
    float Id_ref;           //d轴电流参考值 (Id=0)
    float Iq_ref;           //q轴电流参考值
    
    float err_Id;           //d轴电流误差值
    float err_Iq;           //q轴电流误差值

    float Ki;
    float Kp;
    
    float Id_KI_sum;        //Id积分累计
    float Iq_KI_sum;        //Iq积分累计

}PI_CURRENT_TypeDef;

//▲速度PI控制器结构体
typedef struct
{
    float speed_ref;        //参考速度值 RPM
    
    float err_speed;        //速度误差值 RPM

    float Ki;
    float Kp;
    
    float speed_KI_sum;        //Id积分累计

}PI_SPEED_TypeDef;

//▲MTPA用到电机参数结构体
typedef struct
{
    float Rs ;
    float Ls ;
    float phi_f;
    float Ld ;
    float Lq ;

}MTPA_TypeDef;



extern FOC_TypeDef        MyFoc;
extern PI_CURRENT_TypeDef C_PI;
extern PI_SPEED_TypeDef   S_PI;


/****▲▲函数声明▲▲****/

//▲FOC过程函数
float My_limit(float *limit, float limit_max, float limit_min);
float Normalize_theta(float theta);
void  Clarke(FOC_TypeDef *Foc);
void  Park(FOC_TypeDef *Foc , float theta);
void  Invpark(FOC_TypeDef *Foc , float theta);
void  Svpwm(FOC_TypeDef *Foc);

//▲FOC实现综合函数
void  VF_OpenLoop(FOC_TypeDef *Foc, float Ud, float Uq, float theta);
void  IF_OpenLoop(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI, float IU, float IV, float IW, float Iq_ref, float theta);
void  CurrentLoop_Encode(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI, float IU, float IV, float IW, float Iq_ref, float theta);
void  SMO_C_Control(FOC_TypeDef *Foc, PI_CURRENT_TypeDef *PI, float IU, float IV, float IW, float Iq_ref, float theta);
void  SMO_S_C_Control(FOC_TypeDef *Foc,PI_SPEED_TypeDef *S_PI, PI_CURRENT_TypeDef *C_PI, float IU, float IV, float IW, float Speed_ref, float theta);

//▲控制器实现函数
void  CurrentPI(FOC_TypeDef *Foc , PI_CURRENT_TypeDef *PI);
void  SpeedPI  (FOC_TypeDef *Foc , PI_SPEED_TypeDef   *PI , float *Iqref);

//▲其他控制策略函数
void  MTPA_Calculate(MTPA_TypeDef *MTPA ,PI_CURRENT_TypeDef *PI);



#endif
