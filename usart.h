/**
  ******************************************************************************
  * File Name          : USART.h
  * Description        : This file provides code for the configuration
  *                      of the USART instances.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __usart_H
#define __usart_H
#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

typedef enum{
	NUNE			= 0,
	
	Begin			= 2,

	BearingOnRoller=0x0B,//����ڹ����� 

	BearingPlatformOk = 0x0F,	//���ƽ̨OK
	CameraAdjustOk=0x10,  //���λ�õ���ָʾ
	AdjustAllDone = 0x13,    //�û�������в����� ��������λ����ȫ��OK
		
	CameraGrabStart = 0x21, //�����п���
	
	Connecting = 0x30,
	MasterConnected = 0x31, //��ʾ�������ӳɹ�

	BearingPlatformAccurate=0x63,//�����������ƽ̨���˾�ȷ��λ
	BPlatform_AdjustDone = 0x64, //��е���λ�����
		
	RollerSetStart=0x95,
	RollerSetStop = 0x96,
	
	GuardSet = 0x97, 
	
	CameraHome_Start = 0xB0, //���ƽ̨�سǿ�ʼ	
	BPlatformHome_Start = 0xB1,//���ƽ̨�سǿ�ʼ
	
	CameraHome_Done = 0xB2,  //���ƽ̨����س����
	BPlatformHome_Done = 0xB3, //���ƽ̨����س����
	
	CameraGrabDone = 0xD9, //�����պ�
	
	BeginSignGot=0xDB,
	///////////////////////////////////////////////////////////////////////////////////////////////////
	P1_CameraMotorAccurate = 0xA1,
	P2_CameraMotorAccurate = 0xA2,
	P3_CameraMotorAccurate = 0xA3,

	
	P1_CameraMotorAdjustDone = 0xA5,
	P2_CameraMotorAdjustDone = 0xA6,
	P3_CameraMotorAdjustDone = 0xA7,
	P1R_CameraMotorAdjustDone = 0xA8,
	
	ReloadBearing = 0xA6,
	
	MCU_Reset = 0x15,	
	PreAdjust = 0x16,     //����س�Ԥ��λ
	PreAdjustDone =0x18,
	
	module_test = 0x17,
}Msg_Typedef;



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
// Э����ض���
#define FRAME_LENTH               	28    // ָ���
#define FRAME_START               	0xAA  // ����֡��ʼ
#define FRAME_END                 	'/'   // ����֡����
#define FRAME_CHECK_BEGIN          	1    // У�鿪ʼ��λ�� RxBuf[1]
#define FRAME_CHECKSUM            	26    // �������λ��   RxBuf[26]
#define FRAME_CHECK_NUM           	25    // ��ҪУ����ֽ���
#define FILL_VALUE                	0x55  // ���ֵ

#define MSG_RX_SUCCESS_0			0x65
#define MSG_RX_SUCCESS_1			0x66	// �а�	���ճɹ�
#define MSG_RX_SUCCESS_2			0x67	// �а����� ���ճɹ�
#define MSG_RX_BUSY_0				0x86
#define MSG_RX_BUSY_1				0x87	// �а� �豸��æ
#define MSG_RX_BUSY_2				0x88	// �а����� �豸��æ

/* USER CODE END Private defines */

void MX_UART5_UART_Init(void);
void MX_USART1_UART_Init(void);

/* USER CODE BEGIN Prototypes */
uint8_t CheckSum(uint8_t *Ptr,uint8_t Num );
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ usart_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
