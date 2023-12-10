#ifndef __BSP_CONFIG_H_
#define __BSP_CONFIG_H_




/**************** CAN  part ****************/
#define CONFIG_CAN_FIFO_ENABLE 0

#define CONFIG_CAN1_FIFO_ENABLE 0
#define CONFIG_CAN2_FIFO_ENABLE 0

//CAN FIFO ��������
//����FIFO�ĵ�Ԫ������Ҫ2���ݴη�
#define CAN1_TX_FIFO_UNIT_NUM (256)
#define CAN2_TX_FIFO_UNIT_NUM (256)

//CAN���ջص�����ָ�롣
// ��Σ�CAN_RxHeaderTypeDef *header, uint8_t *data
// out: void
#define ECF_CAN1_Rx_Callback_Fun NULL
#define ECF_CAN2_Rx_Callback_Fun NULL

#define CONFIG_CAN_SEND_MOTOR_USE_FIFO 0



/**************** DR16 part ****************/
#define CONFIG_DR16_USART_HANDLE huart3

#define DR16_LOST_DETECT_ENABLE 1

#if DR16_LOST_DETECT_ENABLE
/* �������ʱ��û���յ��µ�ң�������ݾ���Ϊ�Ѿ�ʧ�� */
#define REMOTE_LOST_TIME ((uint32_t)50) //50ms
#endif

/**************** Referee part ****************/
#define CONFIG_REFEREE_USART_HANDLE huart6

#endif

