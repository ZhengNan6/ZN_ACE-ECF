#include "robot_cmd.h"
#include "bsp_dr16.h"
#include "maths.h"
#include "pid.h"
#include "bsp_referee.h"
#include "imu_task.h"
#include "bsp_Motor_Encoder.h"
#include <stdbool.h>
Robot_cmd_t Robot_cmd;

/*该文件内私密变量*/
static uint8_t init_flag = 0;
/*底盘运动所需的各种数据*/
static const REFEREE_t   *referee_cmd;
static const RC_ctrl_t   *RC_ctrl;
static const fp32        *Difference_Angle_between_Chassis_Gimbal;
static const INS_t       *ins;
static pid_parameter_t   chassis_follow_gambal_speed;


/**
  *@brief 返回底盘运动命令数据指针
  */
Robot_cmd_t* get_Robot_cmd_point(void)
{
    if (init_flag++ == 0) CMD_Init();
    return &Robot_cmd;
}

/**
  *@brief 获取底盘运动命令所需的各种数据指针
  */
void CMD_Init(void)
{
    fp32 close = 0;
    Difference_Angle_between_Chassis_Gimbal = &close;
    RC_ctrl = RC_Get_RC_Pointer();
    Robot_cmd.Chassis_Mode = NO_FOLLOW;
    ins = get_imu_control_point();
}

void Mode_Set(void)
{
    static Chassis_Mode_t last_Chassis_Mode = NO_FOLLOW;
    static Chassis_Mode_t RC_Chassis_Mode = NO_FOLLOW;//模式数据，用于判断有无状态改变
    
    if (RC_ctrl->rc.s2 == RC_SW_DOWN){//遥控器左侧开关状态为[下]
        Robot_cmd.Chassis_Mode = NO_FORCE;//无行动模式
        return;
    }
    switch(RC_ctrl->rc.s1)//遥控左侧开关
    {
        case RC_SW_DOWN://遥控器左侧开关状态为[下]
            RC_Chassis_Mode = NO_FOLLOW;//底盘不跟随云台
            break;
        case RC_SW_MID://遥控器左侧开关状态为[中]
            RC_Chassis_Mode = FOLLOW;//底盘跟随云台
            break;
        case RC_SW_UP://遥控器左侧开关状态为[上]
            RC_Chassis_Mode = SPIN;//底盘小陀螺模式
            break;
        default://啥模式都没给，防止出错，无行动模式
            RC_Chassis_Mode = NO_FORCE;
            break;
    }
    //检测到改变后再设置Robot_cmd.Chassis_Mode，避免用键盘设置的模式数据在松开按键后被手柄的原先模式数据覆盖
    if(last_Chassis_Mode != RC_Chassis_Mode)
    {
        Robot_cmd.Chassis_Mode = RC_Chassis_Mode;
    }
    last_Chassis_Mode = RC_Chassis_Mode;
    
    if(RC_ctrl->kb.bit.Q == 1)  Robot_cmd.Chassis_Mode = FOLLOW;
    if(RC_ctrl->kb.bit.E == 1)  Robot_cmd.Chassis_Mode = SPIN;
    if(RC_ctrl->kb.bit.R == 1)  Robot_cmd.Chassis_Mode = NO_FOLLOW;
}

static void NO_FORCE_Mode()
{
	Robot_cmd.Speed_set.chassis_x	=	0;
    Robot_cmd.Speed_set.chassis_y	=	0;
	Robot_cmd.Speed_set.chassis_z	=	0;
}

static void NO_FOLLOW_Mode()
{
	Robot_cmd.Speed_set.chassis_z = 0;

	Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(*Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(*Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(*Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(*Difference_Angle_between_Chassis_Gimbal);
}

static void FOLLOW_Mode()
{
	Robot_cmd.Speed_set.chassis_z = PidCalculate(&chassis_follow_gambal_speed, 0, *Difference_Angle_between_Chassis_Gimbal);
    
    Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(*Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(*Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(*Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(*Difference_Angle_between_Chassis_Gimbal);
}

static void SPIN_Mode()
{
    //根据英雄等级选择自旋速度
	switch(referee_cmd->Robot_Status.robot_level)
	{
		case 1:
			Robot_cmd.Speed_set.chassis_z	=	6000;
			break;
		case 2:
			Robot_cmd.Speed_set.chassis_z	=	4000;
			break;
		case 3:
			Robot_cmd.Speed_set.chassis_z	=	6000;
            break;
		default:
			Robot_cmd.Speed_set.chassis_z	=	3000 ;
			break;
	}
	Robot_cmd.Speed_set.chassis_x = -Robot_cmd.Speed_set.gambal_y * sin_calculate(*Difference_Angle_between_Chassis_Gimbal)
                                    -Robot_cmd.Speed_set.gambal_x * cos_calculate(*Difference_Angle_between_Chassis_Gimbal);
    Robot_cmd.Speed_set.chassis_y =  Robot_cmd.Speed_set.gambal_y * cos_calculate(*Difference_Angle_between_Chassis_Gimbal) 
                                    -Robot_cmd.Speed_set.gambal_x * sin_calculate(*Difference_Angle_between_Chassis_Gimbal);
}
/**
  *@note 需要注意的是速度设定为轮子的向车头前的线速度,实际左侧电机偏置安装需要电流取反，在后续的电流计算中处理
  */
/*
  LF  /   \  RF
        ^
        |
  LB  \   /  RB
*/
void Speed_Set(void)
{
    //向前为Y轴正向，向右为X轴正向
    //云台坐标系
    Robot_cmd.Speed_set.gambal_x =  ((RC_ctrl->rc.ch[0] ) + (-RC_ctrl->kb.bit.A + RC_ctrl->kb.bit.D) * 660 ) * K_FULL_SPEED_SET;
    Robot_cmd.Speed_set.gambal_y = -((RC_ctrl->rc.ch[1] ) + (-RC_ctrl->kb.bit.S + RC_ctrl->kb.bit.W) * 660 ) * K_FULL_SPEED_SET;
    //云台坐标系速度分解到底盘坐标系
    switch(Robot_cmd.Chassis_Mode)
	{
        case NO_FOLLOW:
			NO_FOLLOW_Mode();//底盘不跟随云台
			break;
        case FOLLOW:
			FOLLOW_Mode();//底盘跟随云台
			break;
		case SPIN:
			SPIN_Mode();  //底盘自旋
			break;			
		case NO_FORCE:
			NO_FORCE_Mode();//不动
			break;
	}
    //麦轮解算
//    这一版左侧的电机数值取反
//    Robot_cmd.Speed_set.LF_motor = -Robot_cmd.Speed_set.chassis_x - Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * LF_CENTER;
//    Robot_cmd.Speed_set.RF_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RF_CENTER;
//	  Robot_cmd.Speed_set.LB_motor =  Robot_cmd.Speed_set.chassis_x - Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * LB_CENTER;
//    Robot_cmd.Speed_set.RB_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RB_CENTER;
    Robot_cmd.Speed_set.LF_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y + Robot_cmd.Speed_set.chassis_z * LF_CENTER;
    Robot_cmd.Speed_set.RF_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RF_CENTER;
    Robot_cmd.Speed_set.LB_motor = -Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y + Robot_cmd.Speed_set.chassis_z * LB_CENTER;
    Robot_cmd.Speed_set.RB_motor =  Robot_cmd.Speed_set.chassis_x + Robot_cmd.Speed_set.chassis_y - Robot_cmd.Speed_set.chassis_z * RB_CENTER;
}

void State_Set(void)
{
    Robot_cmd.Chassis_State = SPEED;  // 默认不锁车
    if ( Robot_cmd.Chassis_Mode == FOLLOW || Robot_cmd.Chassis_Mode == NO_FOLLOW )
    {
        //摇杆速度死区限制
        if ( abs(Robot_cmd.Speed_set.chassis_x) > SPEED_DEADBAND || abs(Robot_cmd.Speed_set.chassis_y) > SPEED_DEADBAND )
            return;
        //陀螺仪加速度检测，这样做防止高速下遥控突然回正导致直接刹车锁死
        if( abs(ins->E_Accel[X]) >= 0.1f || abs(ins->E_Accel[Y]) >= 0.1f) 
            return;
        
        Robot_cmd.Chassis_State = LOCK_POSITION;
    }      
}


void Chassis_cmd_set(void)
{
    Mode_Set();
    Speed_Set();
    State_Set();
}

void Fire_Set(void)
{
    static bool rc_4_reset = true;
    if (Robot_cmd.Fire_State == On_Fire ||  Robot_cmd.Fire_State == On_Empty)  return;//防止打断拨弹盘动作

    switch(RC_ctrl->rc.s2){//遥控右侧开关
        case RC_SW_DOWN://开关状态为[下]
            Robot_cmd.Fire_State = NO_Fire_FORCE;//关摩擦轮
            break;
        case RC_SW_MID://开关状态为[中]
            Robot_cmd.Fire_State = NO_Fire_FORCE;//关摩擦轮
            break;
        case RC_SW_UP: //开关状态为[上]
            Robot_cmd.Fire_State = Ready_Fire;   //开摩擦轮
            break;
        default:
            Robot_cmd.Fire_State = NO_Fire_FORCE;//关摩擦轮
            break;
        }
    //拨杆回正
    if ((RC_ctrl->rc.ch[4] > -200 &&  RC_ctrl->rc.ch[4] <200)|| RC_ctrl->mouse.press_l == 1)    rc_4_reset = true;
    
    if (rc_4_reset == false)    return;//防止连发
    if (Robot_cmd.Fire_State == Ready_Fire && (RC_ctrl->rc.ch[4] >= 650 || RC_ctrl->mouse.press_l == 1))
    {
        rc_4_reset = false;
        Robot_cmd.Fire_State = On_Fire;
    }  
    if (RC_ctrl->rc.ch[4] <= -650)   
    {
        rc_4_reset = false;
        Robot_cmd.Fire_State = On_Empty;
    }
}