/************************** Dongguan-University of Technology -ACE**************************
* @file DJI_Motor.c
* @brief  �󽮵������������
* @author zhengNannnn
* @version 1.0
* @date 2023-11-29
* @note ����ǰ��bsp_Motor_Encoder.c
*
* @history
* Date       Version Author Description
* 2023-11-29   1.0   ֣�   ���������Ĵ󽮵����������
* 2023-12-04   1.1   ֣�   ���can�����жϵ��õĻص�������������Safe����ι��
* @verbatim
* ==============================================================================
*  Ŀ��ֵ������ʵ������
*  ��ԭ�ȵĴ󽮵��ʹ�÷�ʽ
*  ���д󽮵������Ϣ���ռ�����һ���������棬��ԭ��ÿʹ��һ���������Ҫ��CAN�ж������һ��������ȷ����˲���
*  �Ż���ԭ����Ҫ�Լ��ֶ���������������͵����������ֻ��һ�������ᷢ������������һ���ĸ��������ȻҲ����ָ��һ���飬Ҳ���Ե���ֻ��һ��
*  ������ԭ��PID���ʹ�÷�ʽ��ֻ��Ҫ�����ṩ�ļ��㺯���Ϳ������PID����
*  �Ͻ��˵�����������͵���ص��Ľ�����Ϣ��GM6020��M2006��M3508���������������ֿ������ڽ����У�������M2006����Ϊ�������
*
*  Demo:
*      
*      DJIMotor_object_t *DJI_Motor;
*      DJI_Motor = DJIMotor_Init(1,1,false,GM6020,168);//����can1��idΪ1������뾶168mm
*      DJI_Motor->Using_PID = Position_Speed_PID;//λ���ٶȻ�
*      DJIMotor_Set_val(DJI_Motor,0);//��0��
*      //PID��ʼ��
*      PidInit(&DJI_Motor->Speed_PID,200,0,0,Output_Limit);//ʹ������޷�
*      PidInitMode(&DJI_Motor->Speed_PID,Output_Limit,5000,0);//����޷�ģʽ����
*      PidInit(&DJI_Motor->Position_PID,10,0,0,Output_Limit);//ʹ������޷�
*      PidInitMode(&DJI_Motor->Position_PID,Output_Limit,1000,0);//����޷�ģʽ����
*      
*      while (1)
*      {
*          DJIMotor_PID_Calc(DJI_Motor);
*          //��ѡһ
*          DJIMotor_Send_Group(0);//����ļ��ڵõ��˵��λ��Ground[0];
*          DJIMotor_Send(DJI_Motor);//���ʹ˵���������
*          DJMotor_Send_only_one(DJI_Motor);//ֻ������һ�����
*      }
*   
*
*  ע�����
*      ��Ҫ��DJIMotor_Call_Back��������CAN�Ľ����жϺ���HAL_CAN_RxFifo0MsgPendingCallback����ȥ,Ŀǰ�ú���д���˽�����ϢΪFIFO0����,���������иı�,��Ҫ�޸�
*      __weak void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)��Ҫ��д����麯����Ŀǰ���ļ��Ѿ�������
*      
*
* ==============================================================================
* @TODO �������ɱ��������������й�PID����,����,ֻ��������µ�Ŀ��ֵ
* @endverbatim
************************** Dongguan-University of Technology -ACE***************************/
#include "DJI_Motor.h"
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdlib.h>
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
static CAN_RxHeaderTypeDef CAN1_Rxmessage;					//������Ϣ�ṹ��
uint8_t  DJI_Motor_Rx_Data[8];    							//�������ݵ�����
CAN_TxHeaderTypeDef CAN_TxHeader_DJI;
uint8_t DJI_Motor_Tx_Data[8];
uint32_t send_mail_box;


DJIMotor_object_t ALL_Motor[2][8];//[2]{0��can1��1��can2}  [8]{���ID��}
// 6020��id 1-4��2006/3508��id 5-8�ᷢ����ͻ
// ��һ�������ر��� ��ª������
Encoder_Type_e RX_ID_205_208_type[2][4] = {{Type_End,Type_End,Type_End,Type_End},
                                           {Type_End,Type_End,Type_End,Type_End}};
/*
 * C610(M2006)/C620(M3508): 0x200,0x1FF;
 * GM6020:                  0x1FF,0x2FF
 * ����(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
 */
typedef struct{
    int16_t* Wait_Send[4];//ָ������ ָ���������ĸ�id 1-4��5-8
}Ground_t;
Ground_t Ground[6];

/**
  *@brief DJIMotor_Init      �󽮵�����벢��ʼ��
  *@parm  hcan               ���ص�can��    1��2
  *@parm  CanId              ���ص�Can id�� 1-8
  *@parm  reverse_flag       �Ƿ�ƫ�ð�װ
  *@parm  Encoder_Type_e     �󽮵���ͺ�
  *@reval DJIMotor_object_t  ��ʼ����Ĵ󽮵����ַ
  *@note  reverse_flag ƫ�ñ�־λ����˼�� ��ȡ������ʱ�����ת��������ʵ���˶������෴��ͨ���������Ҫȡ��
  */
DJIMotor_object_t *DJIMotor_Init(uint8_t hcan, uint8_t CanId, bool reverse_flag, Encoder_Type_e  Encoder_type, uint16_t radius)
{
    if (hcan != 1 && hcan!=2)   return NULL;
    if (CanId > 8 || CanId < 1) return NULL;
    //6020��id 1-4��2006/3508��id 5-8��ͻ���� ����Ƿ�ID��ͻ
    if (Encoder_type == GM6020 && CanId <= 4)
    {
        if (RX_ID_205_208_type[hcan==1?0:1][CanId-1] != Type_End)   return NULL;
        else    RX_ID_205_208_type[hcan==1?0:1][CanId-1] = Encoder_type;
    }
    if (Encoder_type != GM6020 && CanId >= 5)
    {
        if (RX_ID_205_208_type[hcan==1?0:1][CanId-5] != Type_End)   return NULL;
        else    RX_ID_205_208_type[hcan==1?0:1][CanId-5] = Encoder_type;
    }
    
    DJIMotor_object_t *DJI_Motor = &(ALL_Motor[hcan==1?0:1][CanId-1]);//ָ�����ض�Ӧ�ĵ��
    memset(DJI_Motor, 0, sizeof(DJIMotor_object_t));
   /*
* C610(M2006)/C620(M3508): 0x200,0x1FF;
* GM6020:                  0x1FF,0x2FF
* can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
* can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
    */
    switch (Encoder_type)
    {
        case GM6020:{//6020���� ��������ѹ
            if (CanId <= 4) Ground[hcan==1?0:3].Wait_Send[CanId-1] = &DJI_Motor->voltage_input;
            else            Ground[hcan==1?2:5].Wait_Send[CanId-5] = &DJI_Motor->voltage_input;
        }break;
        
        case M2006:
        case M3508:{//�������� �����������
            if (CanId <= 4) Ground[hcan==1?1:4].Wait_Send[CanId-1] = &DJI_Motor->current_input;
            else            Ground[hcan==1?0:3].Wait_Send[CanId-5] = &DJI_Motor->current_input;
        }break;
        
        case Type_End://��Ϊʲô ��warn��˳��
        default:{
        }break;
    }
    //��ʼ������
    DJI_Motor->hcan = (hcan==1? &hcan1:&hcan2);
    DJI_Motor->CanId = CanId;
    DJI_Motor->reverse_flag = reverse_flag;
    DJI_Motor->Encoder_type = Encoder_type;
    Encoder_Init(&DJI_Motor->Motor_encoder, DJI_Motor->Encoder_type, radius);

    return DJI_Motor;
}

/**
  *@brief DJIMotor_CanRx_Callback   �趨�����ж��е��õĺ���
  *@parm  Callback                  void����ֵvoid�����ĺ���ָ��
  */
void DJIMotor_CanRx_Callback(DJIMotor_object_t *DJIMotor, Callback Callback_function)
{
    DJIMotor->CanRxCallBack = Callback_function;
}

/**
  *@brief DJIMotor_Set_val   ����趨Ŀ��ֵ
  *@parm  val                �趨��Ŀ��ֵcm/s���
  */
void DJIMotor_Set_val(DJIMotor_object_t *DJIMotor, fp32 val)
{
    if (DJIMotor->reverse_flag == true)  val = -val;
    if (DJIMotor->Using_PID == Speed_PID)   DJIMotor->set_speed = val;
    else                                    DJIMotor->set_angel = val;
}

/**
  *@brief DJIMotor_Data_Deal ������ݴ���
  *@parm  Motor              ������ĵ��ָ��
  *@parm  Rx_Data            CAN���յ�8��U8����
  */
void DJIMotor_Data_Deal(DJIMotor_object_t *Motor,uint8_t DJI_Motor_Rx_Data[8])
{
    if (Motor->CanRxCallBack != NULL)   Motor->CanRxCallBack();//���ý��ջص�����
    //����������ݴ���
    Motor->Motor_Information.position	    = (int16_t)(DJI_Motor_Rx_Data[0] << 8 | DJI_Motor_Rx_Data[1]);
    Motor->Motor_Information.speed			= (int16_t)(DJI_Motor_Rx_Data[2] << 8 | DJI_Motor_Rx_Data[3]);
    //��ͬ������յ���Ϣ��һ��
    if(Motor->Encoder_type == M2006)Motor->Motor_Information.Output_torque	= (int16_t)(DJI_Motor_Rx_Data[4] << 8 | DJI_Motor_Rx_Data[5]);
    else                            Motor->Motor_Information.Torque_current = (int16_t)(DJI_Motor_Rx_Data[4] << 8 | DJI_Motor_Rx_Data[5]);
	Motor->Motor_Information.temperature	=  DJI_Motor_Rx_Data[6]; 
    //�����̴���
    CAN_DATA_Encoder_Deal(Motor->Motor_Information.position, Motor->Motor_Information.speed, &Motor->Motor_encoder);
}

/**
  *@brief DJIMotor_Call_Back �󽮵���жϻص�����
  *@parm  hcan               ���յ���Ϣ��can����
  *@note  ��Ҫ���ú�������CAN�Ľ����жϺ�������ȥ,Ŀǰ�ú���д���˽�����ϢΪFIFO0����,���������иı�,��Ҫ�޸�
  */
void DJIMotor_Call_Back(CAN_HandleTypeDef *hcan)
{
    if(HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &CAN1_Rxmessage, DJI_Motor_Rx_Data) == HAL_OK)	//��ȡ���յ���Ϣ
	{
        switch(CAN1_Rxmessage.StdId)//��switch����������⴦�������Ӧ������ȥһ��safe����
        {
            case 0x201:
            case 0x202:
            case 0x203:
            case 0x204:
//                if (CAN1_Rxmessage.StdId == 0x203||CAN1_Rxmessage.StdId == 0x204)
//                {
//                    int a = 0;
//                }
                DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN1_Rxmessage.StdId - 0x201], DJI_Motor_Rx_Data);
                break;
            case 0x205://6020��id 1-4��2006/3508��id 5-8��ͻ����
            case 0x206:
            case 0x207:
            case 0x208:
            {
                //���ע���ID�ĵ���ͺ�
                int a = hcan==&hcan1?0:1;
                int b = CAN1_Rxmessage.StdId - 0x205;
                if (RX_ID_205_208_type[hcan==&hcan1?0:1][CAN1_Rxmessage.StdId - 0x205] == GM6020)
                    //6020������id��ӦΪ1-4����ά�±�Ϊ StdId-0x205
                    DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN1_Rxmessage.StdId - 0x205], DJI_Motor_Rx_Data);
                else 
                    //3508 2006 ������id��ӦΪ5-8����ά�±�Ϊ StdId-0x201
                    DJIMotor_Data_Deal(&ALL_Motor[hcan==&hcan1?0:1][CAN1_Rxmessage.StdId - 0x201], DJI_Motor_Rx_Data);
                break;
            }
        }
    }
}

//�������,�����˵���������
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    DJIMotor_Call_Back(hcan);
}

void DJIMotor_Set_speed_val(DJIMotor_object_t *DJIMotor, int16_t val)
{
    DJIMotor->set_speed = val;
}
void DJIMotor_PID_Calc(DJIMotor_object_t *DJIMotor)
{
    switch (DJIMotor->Using_PID) {
        
        case Position_Speed_PID: {
            int16_t temp_PID_ret = motor_position_speed_control(&DJIMotor->Speed_PID,
                                                                &DJIMotor->Position_PID,
                                                                DJIMotor->set_angel,
                                                                DJIMotor->Motor_encoder.Angle,
                                                                DJIMotor->Motor_encoder.linear_speed[1]);
            
            if (DJIMotor->Encoder_type == GM6020)   DJIMotor->voltage_input = temp_PID_ret;
            else                                    DJIMotor->current_input = temp_PID_ret;
        }break;
        
        case Speed_PID:{
            int16_t temp_PID_ret = PidCalculate(&DJIMotor->Speed_PID,
                                                DJIMotor->set_speed, 
                                                DJIMotor->Motor_encoder.linear_speed[1]);
            
            if (DJIMotor->Encoder_type == GM6020)   DJIMotor->voltage_input = temp_PID_ret;
            else                                    DJIMotor->current_input = temp_PID_ret;
        }break;
        
        case No_Current:{
            if (DJIMotor->Encoder_type == GM6020)   DJIMotor->voltage_input = 0;
            else                                    DJIMotor->current_input = 0;
        }break;
        
        default :{
        }break;
    }
}

/**
  *@brif �󽮵���ĸ�һ��������
  *@parm Group_index ���͵�Can����
  *@note can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
  *@note can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
  */
void DJIMotor_Send_Group(uint8_t Group_index)
{
    if (Group_index > 5)    return;
                
    Ground_t * Send_Group;
    CAN_HandleTypeDef *Send_phcan;
    
    switch (Group_index)
    {
        case 0:{
            Send_phcan = &hcan1;
            Send_Group = &Ground[0];
            CAN_TxHeader_DJI.StdId 	= 0x1FF;
            break;
        }
        case 3:{
            Send_phcan = &hcan2;
            Send_Group = &Ground[3];
            CAN_TxHeader_DJI.StdId 	= 0x1FF;
            break;
        }
        case 1:{
            Send_phcan = &hcan1;
            Send_Group = &Ground[1];
            CAN_TxHeader_DJI.StdId 	= 0x200;
            break;
        }
        case 4:{
            Send_phcan = &hcan2;
            Send_Group = &Ground[4];
            CAN_TxHeader_DJI.StdId 	= 0x200;
            break;
        }
        case 2:{
            Send_phcan = &hcan1;
            Send_Group = &Ground[2];
            CAN_TxHeader_DJI.StdId 	= 0x2FF;
            break;
        }
        case 5:{
            Send_phcan = &hcan2;
            Send_Group = &Ground[5];
            CAN_TxHeader_DJI.StdId 	= 0x2FF;
            break;
        }
        
        default:
            Send_Group = NULL;
            break;
    }
	CAN_TxHeader_DJI.IDE = CAN_ID_STD;
	CAN_TxHeader_DJI.RTR = CAN_RTR_DATA;
	CAN_TxHeader_DJI.DLC = 0x08;
    
    if (Send_Group->Wait_Send[0] != NULL){//��ֹ��ָ��
            DJI_Motor_Tx_Data[0] = (*Send_Group->Wait_Send[0]) >> 8;
            DJI_Motor_Tx_Data[1] = (*Send_Group->Wait_Send[0]);
    }else   DJI_Motor_Tx_Data[0] = DJI_Motor_Tx_Data[1] = 0;
    
    if (Send_Group->Wait_Send[1] != NULL){
            DJI_Motor_Tx_Data[2] = (*Send_Group->Wait_Send[1]) >> 8;
            DJI_Motor_Tx_Data[3] = (*Send_Group->Wait_Send[1]);
    }else   DJI_Motor_Tx_Data[2] = DJI_Motor_Tx_Data[3] = 0;    

    if (Send_Group->Wait_Send[2] != NULL){
            DJI_Motor_Tx_Data[4] = (*Send_Group->Wait_Send[2]) >> 8;
            DJI_Motor_Tx_Data[5] = (*Send_Group->Wait_Send[2]);
    }else   DJI_Motor_Tx_Data[4] = DJI_Motor_Tx_Data[5] = 0;

    if (Send_Group->Wait_Send[3] != NULL){//��ֹ��ָ��
            DJI_Motor_Tx_Data[6] = (*Send_Group->Wait_Send[3]) >> 8;
            DJI_Motor_Tx_Data[7] = (*Send_Group->Wait_Send[3]);
    }else   DJI_Motor_Tx_Data[6] = DJI_Motor_Tx_Data[7] = 0;

	HAL_CAN_AddTxMessage(Send_phcan, &CAN_TxHeader_DJI, DJI_Motor_Tx_Data, &send_mail_box);			
}
/*
* C610(M2006)/C620(M3508): 0x200,0x1FF;
* GM6020:                  0x1FF,0x2FF
* can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
* can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
*/
void DJIMotor_Send(DJIMotor_object_t *DJIMotor)
{
    uint8_t Motor_Gounp_index = 1;
    if (DJIMotor->hcan == &hcan2)    Motor_Gounp_index+=3;
    if (DJIMotor->CanId > 4)
    {
        if (DJIMotor->Encoder_type == GM6020)   Motor_Gounp_index+=2;
        else                                    Motor_Gounp_index-=1;
    }
    else
    {
        if (DJIMotor->Encoder_type == GM6020)   Motor_Gounp_index-=2;
        else                                    Motor_Gounp_index+=0;
    }
    DJIMotor_Send_Group(Motor_Gounp_index);
}
/*
* C610(M2006)/C620(M3508): 0x200,0x1FF;
* GM6020:                  0x1FF,0x2FF
* can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
* can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
*/
void DJMotor_Send_only_one(DJIMotor_object_t *DJIMotor)
{
    DJI_Motor_Tx_Data[0] = 0;
    DJI_Motor_Tx_Data[1] = 0;
    DJI_Motor_Tx_Data[2] = 0;
    DJI_Motor_Tx_Data[3] = 0;
    DJI_Motor_Tx_Data[4] = 0;
    DJI_Motor_Tx_Data[5] = 0;
    DJI_Motor_Tx_Data[6] = 0;
    DJI_Motor_Tx_Data[7] = 0;
    int16_t Send_Message;
    if (DJIMotor->Encoder_type == GM6020)   Send_Message = DJIMotor->voltage_input;
    else                                    Send_Message = DJIMotor->current_input;
    
    if (DJIMotor->CanId <= 4)
    {
        if (DJIMotor->Encoder_type == GM6020)   CAN_TxHeader_DJI.StdId = 0x1FF;
        else                                    CAN_TxHeader_DJI.StdId = 0x200;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-1)*2] = Send_Message >> 8;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-1)*2+1] = Send_Message >> 8;
    }
    else
    {
        if (DJIMotor->Encoder_type == GM6020)   CAN_TxHeader_DJI.StdId = 0x2FF;
        else                                    CAN_TxHeader_DJI.StdId = 0x1FF;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-5)*2] = Send_Message >> 8;
        DJI_Motor_Tx_Data[(DJIMotor->CanId-5)*2+1] = Send_Message >> 8;
    }
	CAN_TxHeader_DJI.IDE = CAN_ID_STD;
	CAN_TxHeader_DJI.RTR = CAN_RTR_DATA;
	CAN_TxHeader_DJI.DLC = 0x08;
    
	HAL_CAN_AddTxMessage(DJIMotor->hcan, &CAN_TxHeader_DJI, DJI_Motor_Tx_Data, &send_mail_box);	
}
