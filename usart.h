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

	BearingOnRoller=0x0B,//轴承在滚轴上 

	BearingPlatformOk = 0x0F,	//轴承平台OK
	CameraAdjustOk=0x10,  //相机位置调整指示
	AdjustAllDone = 0x13,    //用户输入轴承参数后 各部门走位都已全部OK
		
	CameraGrabStart = 0x21, //流程中开拍
	
	Connecting = 0x30,
	MasterConnected = 0x31, //表示主板连接成功

	BearingPlatformAccurate=0x63,//轴承升降收扩平台顶杆精确走位
	BPlatform_AdjustDone = 0x64, //轴承调整位置完毕
		
	RollerSetStart=0x95,
	RollerSetStop = 0x96,
	
	GuardSet = 0x97, 
	
	CameraHome_Start = 0xB0, //相机平台回城开始	
	BPlatformHome_Start = 0xB1,//轴承平台回城开始
	
	CameraHome_Done = 0xB2,  //相机平台电机回城完成
	BPlatformHome_Done = 0xB3, //轴承平台电机回城完成
	
	CameraGrabDone = 0xD9, //拍完照后
	
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
	PreAdjust = 0x16,     //电机回城预走位
	PreAdjustDone =0x18,
	
	module_test = 0x17,
}Msg_Typedef;



/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;

/* USER CODE BEGIN Private defines */
// 协议相关定义
#define FRAME_LENTH               	28    // 指令长度
#define FRAME_START               	0xAA  // 数据帧开始
#define FRAME_END                 	'/'   // 数据帧结束
#define FRAME_CHECK_BEGIN          	1    // 校验开始的位置 RxBuf[1]
#define FRAME_CHECKSUM            	26    // 检验码的位置   RxBuf[26]
#define FRAME_CHECK_NUM           	25    // 需要校验的字节数
#define FILL_VALUE                	0x55  // 填充值

#define MSG_RX_SUCCESS_0			0x65
#define MSG_RX_SUCCESS_1			0x66	// 夹板	接收成功
#define MSG_RX_SUCCESS_2			0x67	// 夹板升降 接收成功
#define MSG_RX_BUSY_0				0x86
#define MSG_RX_BUSY_1				0x87	// 夹板 设备繁忙
#define MSG_RX_BUSY_2				0x88	// 夹板升降 设备繁忙

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
