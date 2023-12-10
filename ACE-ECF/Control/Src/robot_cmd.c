#include "robot_cmd.h"
#include "bsp_dr16.h"
#include "maths.h"
#include "pid.h"
#include "bsp_referee.h"
#include "imu_task.h"
#include "bsp_Motor_Encoder.h"
#include <stdbool.h>
Robot_cmd_t Robot_cmd;

/*���ļ���˽�ܱ���*/
static uint8_t init_flag = 0;
/*�����˶�����ĸ�������*/
static const REFEREE_t   *referee_cmd;
static const RC_ctrl_t   *RC_ctrl;
static const fp32        *Difference_Angle_between_Chassis_Gimbal;
static const INS_t       *ins;
static pid_parameter_t   chassis_follow_gambal_speed;


/**
  *@brief ���ص����˶���������ָ��
  */
Robot_cmd_t* get_Robot_cmd_point(void)
{
    if (init_flag++ == 0) CMD_Init();
    return &Robot_cmd;
}

/**
  *@brief ��ȡ�����˶���������ĸ�������ָ��
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
    static Chassis_Mode_t RC_Chassis_Mode = NO_FOLLOW;//ģʽ���ݣ������ж�����״̬�ı�
    
    if (RC_ctrl->rc.s2 == RC_SW_DOWN){//ң������࿪��״̬Ϊ[��]
        Robot_cmd.Chassis_Mode = NO_FORCE;//���ж�ģʽ
        return;
    }
    switch(RC_ctrl->rc.s1)//ң����࿪��
    {
        case RC_SW_DOWN://ң������࿪��״̬Ϊ[��]
            RC_Chassis_Mode = NO_FOLLOW;//���̲�������̨
            break;
        case RC_SW_MID://ң������࿪��״̬Ϊ[��]
            RC_Chassis_Mode = FOLLOW;//���̸�����̨
            break;
        case RC_SW_UP://ң������࿪��״̬Ϊ[��]
            RC_Chassis_Mode = SPIN;//����С����ģʽ
            break;
        default://ɶģʽ��û������ֹ�������ж�ģʽ
            RC_Chassis_Mode = NO_FORCE;
            break;
    }
    //��⵽�ı��������Robot_cmd.Chassis_Mode�������ü������õ�ģʽ�������ɿ��������ֱ���ԭ��ģʽ���ݸ���
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
    //����Ӣ�۵ȼ�ѡ�������ٶ�
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
  *@note ��Ҫע������ٶ��趨Ϊ���ӵ���ͷǰ�����ٶ�,ʵ�������ƫ�ð�װ��Ҫ����ȡ�����ں����ĵ��������д���
  */
/*
  LF  /   \  RF
        ^
        |
  LB  \   /  RB
*/
void Speed_Set(void)
{
    //��ǰΪY����������ΪX������
    //��̨����ϵ
    Robot_cmd.Speed_set.gambal_x =  ((RC_ctrl->rc.ch[0] ) + (-RC_ctrl->kb.bit.A + RC_ctrl->kb.bit.D) * 660 ) * K_FULL_SPEED_SET;
    Robot_cmd.Speed_set.gambal_y = -((RC_ctrl->rc.ch[1] ) + (-RC_ctrl->kb.bit.S + RC_ctrl->kb.bit.W) * 660 ) * K_FULL_SPEED_SET;
    //��̨����ϵ�ٶȷֽ⵽��������ϵ
    switch(Robot_cmd.Chassis_Mode)
	{
        case NO_FOLLOW:
			NO_FOLLOW_Mode();//���̲�������̨
			break;
        case FOLLOW:
			FOLLOW_Mode();//���̸�����̨
			break;
		case SPIN:
			SPIN_Mode();  //��������
			break;			
		case NO_FORCE:
			NO_FORCE_Mode();//����
			break;
	}
    //���ֽ���
//    ��һ�����ĵ����ֵȡ��
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
    Robot_cmd.Chassis_State = SPEED;  // Ĭ�ϲ�����
    if ( Robot_cmd.Chassis_Mode == FOLLOW || Robot_cmd.Chassis_Mode == NO_FOLLOW )
    {
        //ҡ���ٶ���������
        if ( abs(Robot_cmd.Speed_set.chassis_x) > SPEED_DEADBAND || abs(Robot_cmd.Speed_set.chassis_y) > SPEED_DEADBAND )
            return;
        //�����Ǽ��ٶȼ�⣬��������ֹ������ң��ͻȻ��������ֱ��ɲ������
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
    if (Robot_cmd.Fire_State == On_Fire ||  Robot_cmd.Fire_State == On_Empty)  return;//��ֹ��ϲ����̶���

    switch(RC_ctrl->rc.s2){//ң���Ҳ࿪��
        case RC_SW_DOWN://����״̬Ϊ[��]
            Robot_cmd.Fire_State = NO_Fire_FORCE;//��Ħ����
            break;
        case RC_SW_MID://����״̬Ϊ[��]
            Robot_cmd.Fire_State = NO_Fire_FORCE;//��Ħ����
            break;
        case RC_SW_UP: //����״̬Ϊ[��]
            Robot_cmd.Fire_State = Ready_Fire;   //��Ħ����
            break;
        default:
            Robot_cmd.Fire_State = NO_Fire_FORCE;//��Ħ����
            break;
        }
    //���˻���
    if ((RC_ctrl->rc.ch[4] > -200 &&  RC_ctrl->rc.ch[4] <200)|| RC_ctrl->mouse.press_l == 1)    rc_4_reset = true;
    
    if (rc_4_reset == false)    return;//��ֹ����
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