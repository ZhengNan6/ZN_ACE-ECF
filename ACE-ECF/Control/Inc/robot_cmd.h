#ifndef ROBOT_CMD_H
#define ROBOT_CMD_H

#include <struct_typedef.h>
#include "robot_define.h"
#include "bsp_dr16.h"

/* ����robot_def.h�е�macro�Զ�����Ĳ��� */
#define HALF_WHEEL_BASE (WHEEL_BASE / 2.0f)     // �����
#define HALF_TRACK_WIDTH (TRACK_WIDTH / 2.0f)   // ���־�
#define PERIMETER_WHEEL (RADIUS_WHEEL * 2 * PI) // �����ܳ�

#define K_FULL_SPEED_SET    1.0f//�������λ���ٶ�/660
#define SPEED_DEADBAND      5*K_FULL_SPEED_SET

#define DEGREE_2_RAD 0.01745329252f // pi/180
//��һ���õ�Ծ¹���������ģ��
#define LF_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RF_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE - CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define LB_CENTER ((HALF_TRACK_WIDTH + CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)
#define RB_CENTER ((HALF_TRACK_WIDTH - CENTER_GIMBAL_OFFSET_X + HALF_WHEEL_BASE + CENTER_GIMBAL_OFFSET_Y) * DEGREE_2_RAD)

typedef enum
{
    Ready_Fire,   //��Ħ����
    NO_Fire_FORCE,//��Ħ����
    On_Fire,      //������
    On_Empty      //�˵���
}Fire_State_e;

typedef enum
{
		NO_FOLLOW,
		FOLLOW,
		SPIN,
	    NO_FORCE,
}Chassis_Mode_t;

typedef enum
{
	LOCK_POSITION,
	SPEED
}Chassis_State_t;

typedef struct{
    fp32  gambal_x;
    fp32  gambal_y;
    
    fp32  chassis_x;
    fp32  chassis_y;
    fp32  chassis_z;
    
    fp32  RF_motor;
    fp32  RB_motor;
    fp32  LB_motor;
    fp32  LF_motor;
}Speed_set_t;

typedef struct{
    Chassis_Mode_t  Chassis_Mode;
	Chassis_State_t Chassis_State;
    fp32            *Difference_Angle_between_Chassis_Gimbal;
    Speed_set_t     Speed_set;
    Fire_State_e      Fire_State;
}Robot_cmd_t;

void CMD_Init(void);

void Chassis_cmd_set(void);

void Fire_Set(void);

Robot_cmd_t* get_Robot_cmd_point(void);
#endif
