/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
#include "main.h"
#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "math.h"
#include <stdlib.h>
#include "StepMotor/bsp_STEPMOTOR.h"
#include "led/bsp_led.h"

#define  FASTSEEK_SPEED   150		//ԭ��ع��ٶ� BENLAI300
#define  SLOWSEEK_SPEED   80		//ԭ��ع������ٶ� 200

#define DOWN 	0
#define UP		1
#define REUP 2


extern __IO uint32_t uwTick;

// �ٶ����ֵ���������͵����������Щ�����1800����Щ���Դﵽ4000
uint32_t avg_speed_x[3] = {75, 150, 200};
uint32_t avg_speed_y[3] = {75, 150, 200};
uint32_t avg_speed_z[3] = {100, 250, 250};
uint32_t accel_speed_x[3] = {60, 100, 120};                      //{60, 120, 160}
uint32_t accel_speed_y[3] = {60, 100, 120};                      //{60, 120, 160}
uint32_t accel_speed_z[3] = {100, 200, 200};                    //{100, 200, 200}
uint32_t decel_speed_x[3] = {45, 90, 100};                        //{45, 90, 120}
uint32_t decel_speed_y[3] = {45, 90, 100};                        //{45, 90, 120}
uint32_t decel_speed_z[3] = {75,100, 100};                      //{75, 100, 100}

uint32_t set_speed  = 250;         	// �ٶ� ��λΪ0.05rad/sec

// ���ٶȺͼ��ٶ�ѡȡһ�����ʵ����Ҫ��ֵԽ���ٶȱ仯Խ�죬�Ӽ��ٽ׶αȽ϶���
// ���Լ��ٶȺͼ��ٶ�ֵһ������ʵ��Ӧ���жೢ�Գ����Ľ��
uint32_t step_accel = 200;         	// ���ٶ� ��λΪ0.1rad/sec^2
uint32_t step_decel = 150;          // ���ٶ� ��λΪ0.1rad/sec^2



uint32_t fastseek_Speed = FASTSEEK_SPEED;
uint32_t slowseek_Speed = SLOWSEEK_SPEED;

extern __IO uint16_t Toggle_Pulse[4];  //���ƽ̨����ٶȵ���
extern __IO uint8_t  LimNega[3][4];
extern __IO  BeginState_Typedef  BeginState[3][4];
extern __IO uint8_t MotionStatus[3][4];
extern __IO int32_t step_position[3][4];
extern const StepMotor Stepmotor[3][4];

__IO uint32_t pulse_count[4] = {0, 0, 0, 0};		// ��ǰ�������� 0-��е��� 1-�а���չ 2-�а�ߵ� 3- ����

__IO int32_t guardSpr = 0;				//��е��� ������
__IO uint32_t guardlastPulse=0;

extern uint8_t signal[4];			// ���ƽ̨��λ���ı�־λ
int8_t _Plat = -1;
int8_t _Axis = -1;
uint16_t Pin = 0;
uint8_t Dir = 0;
uint8_t flag = 0;

//�����ȷ����ϵ��
uint32_t accurateH=0,accurateT=0,accurateO=0,accurateD=0,accurateP=0;   //��ʮ�� ʮ�� �ٷ�
double accurate_speed[4][4]={0};  //�����ȷ�ٶ�


//���̿�ʼ��ϵ��
__IO uint8_t BeginWorking=0;  //һ�ֵ����̿�ʼ��              �״�Ϊ 0->1 ������Ϊ2->3
__IO uint8_t BearingOnPosition=0; //�����λ�� ������������ֻ����һ��  0 -> �������->1 ��ȱ�ݻ���ȱ�ݺ�-> 2 ���������������� ->3   ����REUP��STOP ->0

__IO double RollerSpeed=0;  //������̵Ĺ����ٶ�


 
 __IO uint32_t Camera_HomeDone[4][4]={0}; //���ƽ̨����ڻسǹ����У��ж���Щ����ѻص����
 
 __IO uint32_t BPlatform_HomeStep=0; //���ƽ̨�سǲ���

 __IO uint32_t ReturnHome_Work[2] = {0};    //�����ԭ������
 
 __IO uint32_t PreAdjusting_Work = 0;
 __IO uint32_t PreAdjust_Step = 0; 
 __IO uint32_t PreAdjust_Done[4][4] = {0};
 uint32_t PreAdjustPosition = 20;
 

 

/* ��չ���� ------------------------------------------------------------------*/
extern uint8_t RxBuf[FRAME_LENTH] ;  // ��λ�����ջ�����
extern __IO uint8_t TxBuf[FRAME_LENTH] ;  // ���ͻ�����

extern __IO Msg_Typedef MSG;


/* Private function prototypes -----------------------------------------------*/
void MsgAnalysis(void);	
	
void SystemClock_Config(void);

void GuardCtrl(uint8_t status);
	
void ReturnStartPointChk(void);  //����س�״�����

void BearingSensorChk(void);      //��鴫������� ��������ع��� 

void SendCmd(uint8_t cmd , uint8_t direction);

void AccuarteMove(uint8_t plat,uint8_t axis);

void RollerSetAndStart(double rSpeed);

void CameraHomeWork(void);
void BPlatformHomeWork(void);
void GuardCtrlWork(void);
void MotorAdjustWork(void);


void CameraHomeSenorChk(void);
void CameraMotorReset(uint8_t plat,uint8_t axis);

void BPlatformAnalysis(uint8_t HomeType);

uint32_t WordsRecognition(uint8_t  word);

//uint8_t Move = 0;
					//����ת�ٵ���
__IO	uint32_t rollerSignBit=0, rollerHundredBit=0, rollerTenBit=0, rollerOneBit=0, rollerDecileBit=0, rollerPercentileBit=0;
__IO	double rollerSetSpeed=0.00;



uint8_t RSxBuf[FRAME_LENTH]; //������λ������


uint8_t TSxBuf[FRAME_LENTH]; //������λ������


__IO uint8_t GuardMoveType = 0;

__IO uint8_t BearingPlatform_AdjustDone= 0;

__IO uint8_t   CameraMotor_AdjustType = 0;
__IO uint32_t CameraMotor_AdjustDone[4][4] = {0}; 


/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
	HAL_Init();

	SystemClock_Config();
	LED_GPIO_Init();
	MX_GPIO_Init();
	MX_TIMx_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_UART5_UART_Init();
	MX_USART1_UART_Init();
	
	if(HAL_UART_Receive_IT(&huart5, (uint8_t *)&RxBuf, FRAME_LENTH))
	{
		Error_Handler();
	}
	
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
	HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_DISABLE);
	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_4, TIM_CCx_DISABLE);
	HAL_GPIO_WritePin(STEPMOTOR1_ENA_PORT, STEPMOTOR1_ENA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPMOTOR2_ENA_PORT, STEPMOTOR2_ENA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPMOTOR3_ENA_PORT, STEPMOTOR3_ENA_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEPMOTOR4_ENA_PORT, STEPMOTOR4_ENA_PIN, GPIO_PIN_RESET);
	
	for(int i=0; i<3; i++)
	{
		for(int j=0; j<4; j++)
		{
			STEPMOTOR_DIR_FORWARD(i, j);
			STEPMOTOR_OUTPUT_ENABLE(i, j);
		}
	}

	while (1)
	{			
		if(ReturnHome_Work[BEARINGPLATFORM_MOTOR] == 1)
		{
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)==1) && (BPlatform_HomeStep == 0) ) //�а巴���ſ���λ
			{
				BPlatformAnalysis(BLOCKEXPEND_1);
				BPlatform_HomeStep = 1;
			}
			if(( HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)==0 ) && (BPlatform_HomeStep == 1) )
			{
				BPlatformAnalysis(BLOCKEXPEND_2); 
				BPlatform_HomeStep = 2;
			}
			if((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)==0) && (BPlatform_HomeStep == 0)) //��ʼ�а����ڴ�����
			{
				BPlatformAnalysis(BLOCKEXPEND_2); 
				BPlatform_HomeStep = 2;				
			}
			
			if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==1)&&(BPlatform_HomeStep==3)) //�а�߶Ȳ��ڴ�����
			{
				BPlatformAnalysis(BLOCKHEIGHT_1);
				BPlatform_HomeStep = 4;	
			}
			if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==0)&&(BPlatform_HomeStep==4)) //�а�߶Ȳ��ڴ�����
			{
				BPlatformAnalysis(BLOCKHEIGHT_2);
				BPlatform_HomeStep = 5;	
			}
			if((HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==0)&&(BPlatform_HomeStep==3)) //�а�߶Ȳ��ڴ�����
			{
				BPlatformAnalysis(BLOCKHEIGHT_2);
				BPlatform_HomeStep = 5;	
			}
			
			if((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==1 )&&(BPlatform_HomeStep ==6 ))   //���˳�ʼ����ԭ��
			{
				BPlatformAnalysis(GUARDHOME_1);
				BPlatform_HomeStep =7;
			}
			if((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0 )&&(BPlatform_HomeStep ==7 ))
			{
				BPlatformAnalysis(GUARDHOME_2);
				BPlatform_HomeStep =0;
				SendCmd(BPlatformHome_Done,COMPUTER);
				ReturnHome_Work[BEARINGPLATFORM_MOTOR] = 0;
			}
			if((HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0 )&&(BPlatform_HomeStep ==6 ))   //���˳�ʼ��ԭ��
			{
				BPlatformAnalysis(GUARDHOME_2);
				BPlatform_HomeStep =0;
				SendCmd(BPlatformHome_Done,COMPUTER);
				ReturnHome_Work[BEARINGPLATFORM_MOTOR] = 0;
			}						
		}
		
		if(ReturnHome_Work[CAMERA_MOTOR] == 1)
		{
			CameraHomeSenorChk();
				
			if((Camera_HomeDone[P1][AXIS_R] == 1)&&(Camera_HomeDone[P1][AXIS_Z] != 1)) //���� ����P1R��������P1Z
			{
				CameraMotorReset(P1,AXIS_Z);								 
			}
				
				//�������ƽ̨����س���ɺ�
			if( (Camera_HomeDone[P2][AXIS_Y]==1) && (Camera_HomeDone[P2][AXIS_X] == 1)
			&& (Camera_HomeDone[P3][AXIS_Y]==1) && (Camera_HomeDone[P3][AXIS_X]==1) && (Camera_HomeDone[P3][AXIS_Z]==1)
			&& (Camera_HomeDone[P1][AXIS_R]==1) && (Camera_HomeDone[P1][AXIS_Y]==1) && (Camera_HomeDone[P1][AXIS_Z]==1) && (Camera_HomeDone[P1][AXIS_X]==1)
		  && (Camera_HomeDone[P3][AXIS_R]==1))
			{
				ReturnHome_Work[CAMERA_MOTOR] = 0; 
				SendCmd(CameraHome_Done,COMPUTER);
			}			
		}
		
		if(PreAdjusting_Work == 1)
		{
			if(PreAdjust_Step == 0)
			{
				STEPMOTOR_AxisMoveRel(P3, AXIS_R, MOTOR_DIR_CCW*10*UNIT_STEP_MM, step_accel, step_decel, set_speed);
				
				STEPMOTOR_AxisMoveAbs(P2, AXIS_X, PreAdjustPosition*UNIT_STEP_MM, accel_speed_x[P2], decel_speed_x[P2], avg_speed_x[P2]);	
				STEPMOTOR_AxisMoveAbs(P2, AXIS_Y,-1*PreAdjustPosition*UNIT_STEP_MM, accel_speed_y[P2], decel_speed_y[P2], avg_speed_y[P2]);
				PreAdjust_Step = 1;
			}
			if((PreAdjust_Step == 1) && (PreAdjust_Done[P3][AXIS_R] == 1) && (PreAdjust_Done[P2][AXIS_X] == 1) && (PreAdjust_Done[P2][AXIS_Y] == 1))
			{
				STEPMOTOR_AxisMoveAbs(P3, AXIS_X, PreAdjustPosition*UNIT_STEP_MM, accel_speed_x[P3], decel_speed_x[P3], avg_speed_x[P3]);
				STEPMOTOR_AxisMoveAbs(P3, AXIS_Y,-1*PreAdjustPosition*UNIT_STEP_MM, accel_speed_y[P3], decel_speed_y[P3], avg_speed_y[P3]);
				STEPMOTOR_AxisMoveAbs(P3, AXIS_Z, -1*PreAdjustPosition * UNIT_STEP_MM, accel_speed_z[2], decel_speed_z[2], avg_speed_z[2]);	
				PreAdjust_Step = 2;
			}
			if( (PreAdjust_Step == 2) && (PreAdjust_Done[P3][AXIS_X] == 1) && (PreAdjust_Done[P3][AXIS_Y] == 1) && (PreAdjust_Done[P3][AXIS_Z] == 1))
			{
				STEPMOTOR_AxisMoveAbs(P1, AXIS_X,PreAdjustPosition*UNIT_STEP_MM, accel_speed_x[P1], decel_speed_x[P1], avg_speed_x[P1]);
				STEPMOTOR_AxisMoveAbs(P1, AXIS_Y, PreAdjustPosition*UNIT_STEP_MM, accel_speed_y[P1], decel_speed_y[P1], avg_speed_y[P1]);	
				STEPMOTOR_AxisMoveAbs(P1, AXIS_Z, -1* PreAdjustPosition * UNIT_STEP_MM, accel_speed_z[0], decel_speed_z[0], avg_speed_z[0]);		
				PreAdjust_Step = 3;
			}
			if((PreAdjust_Step == 3)&&(PreAdjust_Done[P1][AXIS_X] == 1)&&(PreAdjust_Done[P1][AXIS_Y] == 1)&&(PreAdjust_Done[P1][AXIS_Z] == 1))
			{
				STEPMOTOR_AxisMoveAbs(P1, AXIS_R, -1*10*ONE_ANGLEMOVE , step_accel, step_decel, set_speed);
				PreAdjust_Step = 4;
			}
			if((PreAdjust_Step == 4)&&(PreAdjust_Done[P1][AXIS_R] == 1))
			{
				PreAdjusting_Work = 0;
				PreAdjust_Step = 0;
				SendCmd(PreAdjustDone,COMPUTER);				
			}
			
		}

		
    MsgAnalysis();	

		//CameraHomeWork();

		//BPlatformHomeWork();

		//GuardCtrlWork();
		
		MotorAdjustWork();
		
    BearingSensorChk();
	}
}

void MsgAnalysis()
{
	if(MSG != NUNE)
		{
			switch(MSG)
			{				
				case Connecting:    
				{
					 SendCmd(MasterConnected, COMPUTER); 
					break;
				}

				case MCU_Reset:
				{
					__set_FAULTMASK(1); 
					NVIC_SystemReset(); 
					break;
				}
				
				case Begin:
				{		
					SendCmd(BeginSignGot,COMPUTER); 
					BeginWorking=1; //��ʼ�״����̱�־
					GuardCtrl(UP);
					break;
				}


				case CameraGrabDone:
				{
					RollerSetAndStart(4*rollerSetSpeed); 
					HAL_Delay(1000);
					GuardCtrl(UP);	
					BearingOnPosition=2;
					break;
				}

        
								
				case RollerSetStart:      
				{
					rollerSignBit = WordsRecognition(RxBuf[2]);
					rollerHundredBit=WordsRecognition(RxBuf[3]);
					rollerTenBit = WordsRecognition(RxBuf[4]);
					rollerOneBit = WordsRecognition(RxBuf[5]);
					rollerDecileBit = WordsRecognition(RxBuf[6]);
					rollerPercentileBit = WordsRecognition(RxBuf[7]);
					if(rollerSignBit==0)
					{
						rollerSetSpeed=rollerHundredBit*100.00+rollerTenBit*10.00+rollerOneBit*1.00+0.10*rollerDecileBit+0.01*rollerPercentileBit;
				  }
					if(rollerSignBit==1)
					{
						rollerSetSpeed=-1*(rollerHundredBit*100.00+rollerTenBit*10.00+rollerOneBit*1.00+0.10*rollerDecileBit+0.01*rollerPercentileBit);
				  }		
					HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1);
					RollerSetAndStart(4*rollerSetSpeed); 
					break;
				}
				
				case RollerSetStop:
				{
					RollerSetAndStart(0);
					break;
				}
				
				case GuardSet: 
				{
					uint32_t guardHundredBit = 0,guardTenBit = 0,guardOneBit=0, guardDecile = 0,guardPercentileBit = 0;
					double guardMove = 0.00;
					guardHundredBit = WordsRecognition(RxBuf[2]);
					guardTenBit = WordsRecognition(RxBuf[3]);
					guardOneBit=WordsRecognition(RxBuf[4]);
					guardDecile = WordsRecognition(RxBuf[5]);
					guardPercentileBit = WordsRecognition(RxBuf[6]);
					guardMove = guardHundredBit*100.00+guardTenBit*10.00+guardOneBit*1.00+ guardDecile*0.10+guardPercentileBit *0.01;
					guardSpr = (guardMove+90.00+6.00)*(SPR / 360);
					HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
					break;
				}
				
		
					//////////////////////////////////////////////////////////////////////////////////////
							case P1_CameraMotorAccurate:
							{
								switch(RxBuf[3])
								{
									case (ACCURATE_X):
									{
										AccuarteMove(P1,AXIS_X); 					
										STEPMOTOR_AxisMoveAbs(P1, AXIS_X, accurate_speed[P1][AXIS_X]*UNIT_STEP_MM, accel_speed_x[P1], decel_speed_x[P1], avg_speed_x[P1]);
										break;
									}
									case (ACCURATE_Y):
									{
                    AccuarteMove(P1,AXIS_Y); 			
										STEPMOTOR_AxisMoveAbs(P1, AXIS_Y, accurate_speed[P1][AXIS_Y]*UNIT_STEP_MM, accel_speed_y[P1], decel_speed_y[P1], avg_speed_y[P1]);	
										break;
									}
									case (ACCURATE_Z):
									{
									  AccuarteMove(P1,AXIS_Z); 			
										STEPMOTOR_AxisMoveAbs(P1, AXIS_Z, -1* accurate_speed[P1][AXIS_Z] * UNIT_STEP_MM, accel_speed_z[0], decel_speed_z[0], avg_speed_z[0]);		
										break;
									}
									case (ACCURATE_R):
									{
										AccuarteMove(P1,AXIS_R); 		
										STEPMOTOR_AxisMoveAbs(P1, AXIS_R, -1*accurate_speed[P1][AXIS_R]*ONE_ANGLEMOVE , step_accel, step_decel, set_speed);
										break;
									}
								}		
								break;
							}
							
							case P2_CameraMotorAccurate: 
							{
								switch(RxBuf[3])
								{
									case (ACCURATE_X):
									{
										AccuarteMove(P2,AXIS_X); 				
										STEPMOTOR_AxisMoveAbs(P2, AXIS_X, accurate_speed[P2][AXIS_X]*UNIT_STEP_MM, accel_speed_x[P2], decel_speed_x[P2], avg_speed_x[P2]);		
										break;
									}
									case (ACCURATE_Y):
									{
									  AccuarteMove(P2,AXIS_Y); 					 	
								    STEPMOTOR_AxisMoveAbs(P2, AXIS_Y,-1*accurate_speed[P2][AXIS_Y]*UNIT_STEP_MM, accel_speed_y[P2], decel_speed_y[P2], avg_speed_y[P2]);
										/*��ʼ���ƽ̨�������λ��*/
										CameraMotor_AdjustType = 1; 												
										break;
									}
								}
								break;
							}
							case P3_CameraMotorAccurate:
							{
								switch(RxBuf[3])
								{
									case (ACCURATE_X):
									{
										AccuarteMove(P3,AXIS_X); 	
										STEPMOTOR_AxisMoveAbs(P3, AXIS_X, accurate_speed[P3][AXIS_X]*UNIT_STEP_MM, accel_speed_x[P3], decel_speed_x[P3], avg_speed_x[P3]);
										break;
									}
									case (ACCURATE_Y):
									{
									  AccuarteMove(P3,AXIS_Y); 		
										STEPMOTOR_AxisMoveAbs(P3, AXIS_Y, -1*accurate_speed[P3][AXIS_Y]*UNIT_STEP_MM, accel_speed_y[P3], decel_speed_y[P3], avg_speed_y[P3]);
										break;
									}
									case (ACCURATE_Z): 
									{
										AccuarteMove(P3,AXIS_Z); 		
										STEPMOTOR_AxisMoveAbs(P3, AXIS_Z, -1*accurate_speed[P3][AXIS_Z] * UNIT_STEP_MM, accel_speed_z[2], decel_speed_z[2], avg_speed_z[2]);	
										break;
									}
								}
								break;
							}
					
					
					
					
					
					//////////////////////////////////////////////////////////////////////////////////////
					
				case BearingPlatformAccurate:			
				{
					switch(RxBuf[2]) //�ж����ĸ�
					{
						case(BLOCK_EXPAND):  
						{
							uint32_t blockEHundredBit=0,blockETenBit=0,blockEOneBit=0,blockEDecile=0,blockEPercentileBit=0;
							double blockEMove=0.00;
							blockEHundredBit=WordsRecognition(RxBuf[3]);
	     				blockETenBit = WordsRecognition(RxBuf[4]);
		    			blockEOneBit= WordsRecognition(RxBuf[5]);
				      blockEDecile = WordsRecognition(RxBuf[6]);
    					blockEPercentileBit = WordsRecognition(RxBuf[7]);						
							blockEMove=blockEHundredBit*100.00+blockETenBit*10.00+blockEOneBit*1.00+ blockEDecile*0.10+blockEPercentileBit *0.01;
							
							HAL_GPIO_WritePin(STEPMOTOR3_DIR_PORT, STEPMOTOR3_DIR_PIN, GPIO_PIN_SET);
							pulse_count[1]=(uint32_t)(blockEMove*UNIT_STEP_MM*1.25);
							TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_ENABLE);
							break;
						}
						
						case(BLOCK_UPDOWN):  				
						{
							uint32_t blockUDHundredBit=0,blockUDTenBit=0,blockUDOneBit=0,blockUDDecile=0,blockUDPercentileBit=0;
							double blockUDMove=0.00;
							blockUDHundredBit=WordsRecognition(RxBuf[3]);
	     				blockUDTenBit = WordsRecognition(RxBuf[4]);
		    			blockUDOneBit= WordsRecognition(RxBuf[5]);
				      blockUDDecile = WordsRecognition(RxBuf[6]);
    					blockUDPercentileBit = WordsRecognition(RxBuf[7]);						
							blockUDMove=blockUDHundredBit*100.00+blockUDTenBit*10.00+blockUDOneBit*1.00+ blockUDDecile*0.10+blockUDPercentileBit *0.01;
							
		          HAL_GPIO_WritePin(STEPMOTOR4_DIR_PORT, STEPMOTOR4_DIR_PIN, GPIO_PIN_SET);
							pulse_count[2]=(uint32_t)(blockUDMove*UNIT_STEP_MM*5);
            	TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_4, TIM_CCx_ENABLE);				
							break;
						}							
	
						case(TOPBAR_MOVE):  				
						{
							BearingPlatform_AdjustDone = 1;
							
							uint32_t blockTHundredBit=0,blockTTenBit=0,blockTOneBit=0,blockTDecile=0,blockTPercentileBit=0;
							double blockTMove=0.00;
							blockTHundredBit=WordsRecognition(RxBuf[3]);
	     				blockTTenBit = WordsRecognition(RxBuf[4]);
		    			blockTOneBit= WordsRecognition(RxBuf[5]);
				      blockTDecile = WordsRecognition(RxBuf[6]);
    					blockTPercentileBit = WordsRecognition(RxBuf[7]);						
							blockTMove=blockTHundredBit*100.00+blockTTenBit*10.00+blockTOneBit*1.00+ blockTDecile*0.10+blockTPercentileBit *0.01;
							
							pulse_count[3]=(uint32_t)(blockTMove*UNIT_STEP_MM);
							
							STEPMOTOR_AxisMoveRel(P3, AXIS_R, MOTOR_DIR_CCW*pulse_count[3], step_accel, step_decel, set_speed);
							break;
						}											
					}
					break;
				}				
	
				case CameraHome_Start: 
				{
					ReturnHome_Work[CAMERA_MOTOR] = 1;            //���ƽ̨�������㿪������
					//P2ƽ̨			
					if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)==1)   //P2Y��
					 {
					   	 Camera_HomeDone[P2][AXIS_Y] = 0;
						   CameraMotorReset(P2,AXIS_Y);
					 }
						if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12)==1)   //P2X��
					 {
						 	  Camera_HomeDone[P2][AXIS_X] = 0;
						 		CameraMotorReset(P2,AXIS_X);
					 }					
					
					 //P3ƽ̨
					if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10)==1)   //P3Y��
					 {
					   	  Camera_HomeDone[P3][AXIS_Y] = 0;				
                CameraMotorReset(P3,AXIS_Y);						 
					 }
					if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)==1)   //P3X��
					 {
					   	  Camera_HomeDone[P3][AXIS_X] = 0;			
                CameraMotorReset(P3,AXIS_X);	
					 }
					if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0)==1)   //P3Z��
					 {
					   	  Camera_HomeDone[P3][AXIS_Z] = 0;	
                CameraMotorReset(P3,AXIS_Z);							 
					 }			
					if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_2)==1)   //P3R�ᶥ�˵��
					 {
					   	  Camera_HomeDone[P3][AXIS_R] = 0;	
                CameraMotorReset(P3,AXIS_R);							 
					 }							 
					 
					 //P1ƽ̨
					if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7)==1)    //P1R��
					{
					   	 Camera_HomeDone[P1][AXIS_R] = 0;	
               CameraMotorReset(P1,AXIS_R);							
					}
					if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3)==1)   //P1Y��
					 {
			  	    	Camera_HomeDone[P1][AXIS_Y] = 0;	
                CameraMotorReset(P1,AXIS_Y);							 
					 }
					 
						if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4)==1)   //P1Z��
					 {
			  	    	Camera_HomeDone[P1][AXIS_Z] = 0;				 
					 }		
					 
					if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1)==1)   //P1X��
					 {
			  	    	Camera_HomeDone[P1][AXIS_X] = 0;				
                CameraMotorReset(P1,AXIS_X);								 
					 }						 
					break;
				}
				

				case BPlatformHome_Start:
				{
					ReturnHome_Work[BEARINGPLATFORM_MOTOR] = 1;
					break;
				}
				
				case PreAdjust:
				{
					
					break;
				}
				
				case module_test:
				{
					HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
					guardSpr = 800;
					GuardCtrl(UP);
					break;
				}
				
					default:
					break;
			}
		}
		MSG = NUNE;	
	return;
}

void BPlatformAnalysis(uint8_t HomeType)
{
	switch(HomeType)
	{
		case BLOCKEXPEND_1: 
		{
			HAL_GPIO_WritePin(STEPMOTOR3_DIR_PORT, STEPMOTOR3_DIR_PIN, GPIO_PIN_RESET);		
			pulse_count[1] = 210000;
		  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);		
			break;
		}
		
		case  BLOCKEXPEND_2:
		{		
//			pulse_count[1] =0;
//			HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
//			__HAL_GPIO_EXTI_CLEAR_IT(TRIG3_Pin);
			 HAL_GPIO_WritePin(STEPMOTOR3_DIR_PORT, STEPMOTOR3_DIR_PIN, GPIO_PIN_SET);
			pulse_count[1] = 10000;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);		
			break;
		}
		
		case BLOCKHEIGHT_1:
		{
			HAL_GPIO_WritePin(STEPMOTOR4_DIR_PORT, STEPMOTOR4_DIR_PIN, GPIO_PIN_RESET);		
			pulse_count[2] = 300000;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);			
			break;
		}
		case BLOCKHEIGHT_2:
		{
//						pulse_count[2] =0;
//			__HAL_GPIO_EXTI_CLEAR_IT(INPUT6_PIN);
//			HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);			
			HAL_GPIO_WritePin(STEPMOTOR4_DIR_PORT, STEPMOTOR4_DIR_PIN, GPIO_PIN_SET);
			pulse_count[2] = 2*UNIT_STEP_MM*5;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
			break;
		}
		
		case GUARDHOME_1:
		{
			HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_RESET);
			pulse_count[0] = 8000;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
			break;
		}
		
		case GUARDHOME_2:
		{
			HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
			__HAL_GPIO_EXTI_CLEAR_IT(TRIG2_Pin);							 
			HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_SET);
			pulse_count[0] =0;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);		 
			break;
		}
		
		default:
		break;
	}
		
	return;
}



void CameraHomeSenorChk()  //0427�޸�ʹ�õ�������Ĵ�������Ӧ
{
		//P2ƽ̨			
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_0)==0)   //P2Y��
		 {
          ResponsePoint(P2 , AXIS_Y);			 
          Camera_HomeDone[P2][AXIS_Y] = 1; 	 		 
		 }
			if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_12)==0)   //P2X��
		 {
					ResponsePoint(P2 , AXIS_X);
					Camera_HomeDone[P2][AXIS_X] = 1; 
		 }					
		
		 //P3ƽ̨
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_10)==0)   //P3Y��
		 {
			    ResponsePoint(P3 , AXIS_Y);
					Camera_HomeDone[P3][AXIS_Y] = 1; 
		 }
		if(HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_8)==0)   //P3X��
		 {		 
					ResponsePoint(P3 , AXIS_X);
			    Camera_HomeDone[P3][AXIS_X] = 1; 
		 }
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_0)==0)   //P3Z��
		 {
					ResponsePoint(P3 , AXIS_Z);
					Camera_HomeDone[P3][AXIS_Z] = 1; 
		 }							 
		 if(HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_2)==0) //P3R���˵��
		 {
					ResponsePoint(P3 , AXIS_R);
					Camera_HomeDone[P3][AXIS_R] = 1; 			 
		 }
		 
		 //P1ƽ̨
		if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_7)==0)    //P1R��
		{
					ResponsePoint(P1 , AXIS_R);						
					Camera_HomeDone[P1][AXIS_R] = 1; 
		}
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_3)==0)   //P1Y��
		 {
			    ResponsePoint(P1 , AXIS_Y);	
					Camera_HomeDone[P1][AXIS_Y] = 1; 
		 }
		 
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_4)==0)   //P1Z��
		 {			
					ResponsePoint(P1 , AXIS_Z);	
					Camera_HomeDone[P1][AXIS_Z] = 1; 
		 }		
		 
		if(HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_1)==0)   //P1X��
		 {
					ResponsePoint(P1 , AXIS_X);	
					Camera_HomeDone[P1][AXIS_X] = 1; 
		 }						 	
}

void CameraMotorReset(uint8_t plat,uint8_t axis)
{
	//HAL_TIM_Base_Start(htimx[plat]);
	if(axis==AXIS_X)
	{
		STEPMOTOR_AxisMoveAbs(plat, AXIS_X, -1000*UNIT_STEP_MM, accel_speed_x[plat], decel_speed_x[plat], avg_speed_x[plat]);						
	}
	if(axis==AXIS_Y)
	{
		if(plat!=P1)
		{
		STEPMOTOR_AxisMoveAbs(plat, AXIS_Y, 1000*UNIT_STEP_MM, accel_speed_y[plat], decel_speed_y[plat], avg_speed_y[plat]);					
		}
		if(plat==P1)
		{
		STEPMOTOR_AxisMoveAbs(plat, AXIS_Y, -1000*UNIT_STEP_MM, accel_speed_y[plat], decel_speed_y[plat], avg_speed_y[plat]);		
		}		
	}
	if(axis==AXIS_Z)
	{
		STEPMOTOR_AxisMoveAbs(plat, AXIS_Z, 1000*UNIT_STEP_MM, accel_speed_z[plat], decel_speed_z[plat], avg_speed_z[plat]);		
	}
	if(axis==AXIS_R)
	{
		if(plat==P1)
		{
     STEPMOTOR_AxisMoveAbs(P1, AXIS_R, 360*ONE_ANGLEMOVE , step_accel, step_decel, set_speed);						
		}
		if(plat==P3)
		{
			STEPMOTOR_AxisMoveRel(P3, AXIS_R, MOTOR_DIR_CW*1000*UNIT_STEP_MM, step_accel, step_decel, set_speed);
		}			
	}
}
//�յ���λ���Ļس�ָ����Ȱ�ReturnHome_Work[CAMERA_MOTOR]��Ϊ1�����лسǲŻ��õ��Ĳ��ֿ�����Camera_HomeDone������з��Ѿ�����㡣

void CameraHomeWork()
{
		if(ReturnHome_Work[CAMERA_MOTOR] == 1)
		{
			CameraHomeSenorChk();
				
			if((Camera_HomeDone[P1][AXIS_R] == 1)&&(Camera_HomeDone[P1][AXIS_Z] != 1)) //���� ����P1R��������P1Z
			{
				CameraMotorReset(P1,AXIS_Z);								 
			}
				
				//�������ƽ̨����س���ɺ�
			if( (Camera_HomeDone[P2][AXIS_Y]==1) && (Camera_HomeDone[P2][AXIS_X] == 1)
			&& (Camera_HomeDone[P3][AXIS_Y]==1) && (Camera_HomeDone[P3][AXIS_X]==1) && (Camera_HomeDone[P3][AXIS_Z]==1)
			&& (Camera_HomeDone[P1][AXIS_R]==1) && (Camera_HomeDone[P1][AXIS_Y]==1) && (Camera_HomeDone[P1][AXIS_Z]==1) && (Camera_HomeDone[P1][AXIS_X]==1)
		  && (Camera_HomeDone[P3][AXIS_R]==1))
			{
				ReturnHome_Work[CAMERA_MOTOR] = 0; 
				SendCmd(CameraHome_Done,COMPUTER);
			}			
		}
}

void BPlatformHomeWork()
{
		if(ReturnHome_Work[BEARINGPLATFORM_MOTOR] == 1)
		{
								///////////////////////////////////////////////��ʼ״̬����а��ȵ��δ����������
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)==1) && (BPlatform_HomeStep == 0) )//�а巴���ſ���λ
			{
				HAL_GPIO_WritePin(STEPMOTOR3_DIR_PORT, STEPMOTOR3_DIR_PIN, GPIO_PIN_RESET);		
				pulse_count[1] = 210000;
			  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);		
				BPlatform_HomeStep = 1;
				return;
			}
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)==0) && (BPlatform_HomeStep == 1) ) //��ȼа�����������
			{
				pulse_count[1] =0;
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
				__HAL_GPIO_EXTI_CLEAR_IT(TRIG3_Pin);
				BPlatform_HomeStep = 2;
				return;
			}
						//////////////////////////////////////////////////////�����ʼ״̬��ȼа��ڴ�������
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_10)==0) && (BPlatform_HomeStep == 0) )
			{
				pulse_count[1] =0;
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_3);
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
				BPlatform_HomeStep = 2;
				return;
			}
			
			    /////////////////////////////////////////////////////�а��������
			if(BPlatform_HomeStep == 2)
			{
				HAL_GPIO_WritePin(STEPMOTOR3_DIR_PORT, STEPMOTOR3_DIR_PIN, GPIO_PIN_SET);
				pulse_count[1] = 10000;
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_3);		
				BPlatform_HomeStep = 3;
				return;
			}
			  //////////////////////////////////////////////////////�ȼа��ȼе�λ ��ʼ�а�߶�
			  //////////////////////////////////////////////////////��ʼ״̬����а�߶�δ�����²㴫����
			if( (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==1) && (BPlatform_HomeStep == 4) )//���4-�а壨������������
			{
				HAL_GPIO_WritePin(STEPMOTOR4_DIR_PORT, STEPMOTOR4_DIR_PIN, GPIO_PIN_RESET);		
				pulse_count[2] = 300000;
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
				BPlatform_HomeStep = 5;
				return;
			}
			if( (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==0) && (BPlatform_HomeStep == 5) )
			{
				pulse_count[2] =0;
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_10);
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_4);	
				BPlatform_HomeStep = 6;				
				return;
			}
			/////////////////////////////////////////////////�����ʼ״̬�а�߶������²㴫����
			if( (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_10)==0) && (BPlatform_HomeStep == 4) )
			{
				BPlatform_HomeStep = 6;
				return;
			}				
			
			/////////////////////////////////////////////////////�а�߶ȵ�����ϻ���
			if(BPlatform_HomeStep ==6)
			{
				HAL_GPIO_WritePin(STEPMOTOR4_DIR_PORT, STEPMOTOR4_DIR_PIN, GPIO_PIN_SET);
				pulse_count[2] = 2*UNIT_STEP_MM*5;
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_4);
				BPlatform_HomeStep = 7;
				return;
			}
			
			///////////////////////////////////////////////////////�а�߶Ȼس���� ���˿�ʼ�س�
			////////////////////////////////////////���赵�˳�ʼ״̬û����������
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==1) && (BPlatform_HomeStep == 8) )//�������´�
			{
				HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_RESET);
				pulse_count[0] = 8000;
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);
				BPlatform_HomeStep = 9;
				return;
			}
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0) && (BPlatform_HomeStep == 9) )
			{
				pulse_count[0] = 0;
				HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
				__HAL_GPIO_EXTI_CLEAR_IT(GPIO_PIN_9);			
				BPlatform_HomeStep = 10;
				return;
			}
			/////////////////////////////////////////�����ʼ״̬�����ڴ�������
			if( (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0) && (BPlatform_HomeStep == 8) )
			{
				BPlatform_HomeStep = 10;
				return;
			}			
			
			///////////////////////////////////////////////�����ڴ������� �������ƽ̨�س����
			if(BPlatform_HomeStep == 10)
			{
				BPlatform_HomeStep=0;
				SendCmd(BPlatformHome_Done,COMPUTER);
				ReturnHome_Work[BEARINGPLATFORM_MOTOR] = 0;
				return;
			}			
		}				
}

void GuardCtrlWork()
{
		if(GuardMoveType == GUARDUPING)
		{
			HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_SET);	//����
  		pulse_count[0] = guardSpr;
      TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);
			GuardMoveType = GUARDBUSY_UPING;
			return;
		}
		if(GuardMoveType == GUARDDOWNING)
		{
			HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_RESET);	//����
			pulse_count[0] = 6400;		
			TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);
			GuardMoveType = GUARDBUSY_DOWNING;
			return;
		}
		
		//����DOWN��ֹͣ UP��ֹͣ�ڶ�ʱ������
		if((GuardMoveType == GUARDBUSY_DOWNING) && (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0))
		{
		  HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
			__HAL_GPIO_EXTI_CLEAR_IT(TRIG2_Pin);							 
		 	HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_SET);
			pulse_count[0] =0;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);	
			GuardMoveType = GUARDSTANDBY;
			return;
		}
}

void MotorAdjustWork()
{
		if( BearingPlatform_AdjustDone== 2 )
		{
			SendCmd(BPlatform_AdjustDone,COMPUTER);
			BearingPlatform_AdjustDone= 0;
			return;
		}
		
		//���ƽ̨�����ʼ����λ��
		if( (CameraMotor_AdjustType == 1) && (CameraMotor_AdjustDone[P2][AXIS_X] == 1) && (CameraMotor_AdjustDone[P2][AXIS_Y] == 1) )
		{
			SendCmd(P2_CameraMotorAdjustDone,COMPUTER);
			CameraMotor_AdjustType = 2;		
			return;
		}
		
		if( (CameraMotor_AdjustType == 2) && (CameraMotor_AdjustDone[P3][AXIS_X] == 1) && (CameraMotor_AdjustDone[P3][AXIS_Y] == 1) && (CameraMotor_AdjustDone[P3][AXIS_Z] == 1) )
		{
			SendCmd(P3_CameraMotorAdjustDone,COMPUTER);
			CameraMotor_AdjustType = 3;
			return;
		}
		
		if((CameraMotor_AdjustType == 3) && (CameraMotor_AdjustDone[P1][AXIS_X] == 1) && (CameraMotor_AdjustDone[P1][AXIS_Y] == 1) && (CameraMotor_AdjustDone[P1][AXIS_Z] == 1) )
		{
			SendCmd(P1_CameraMotorAdjustDone,COMPUTER);
			CameraMotor_AdjustType = 4;
			return;
		}
		
		if((CameraMotor_AdjustType == 4) && (CameraMotor_AdjustDone[P1][AXIS_R] == 1) )
		{
			SendCmd(P1R_CameraMotorAdjustDone,COMPUTER); 
			CameraMotor_AdjustType = 0;
			return;
		}		
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
	RCC_OscInitTypeDef RCC_OscInitStruct = {0};
	RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
	RCC_OscInitStruct.OscillatorType 	= RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState 			= RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState 		= RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource 	= RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM 			= 8;
	RCC_OscInitStruct.PLL.PLLN 			= 336;
	RCC_OscInitStruct.PLL.PLLP 			= RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ 			= 4;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}
  /** Initializes the CPU, AHB and APB busses clocks 
  */
	RCC_ClkInitStruct.ClockType 		= RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
										|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource 		= RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider 	= RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider 	= RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider 	= RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
	{
		Error_Handler();
	}

}




void BearingSensorChk() 
{
		if((HAL_GPIO_ReadPin(TRIG1_GPIO_Port, TRIG1_Pin)==0)&&(BeginWorking==1))   //�����������ƽ̨�ϴ�����
		 {
    		if(BearingOnPosition==0)
				{
					HAL_Delay(400);
					GuardCtrl(DOWN);		
					BearingOnPosition=1; //���� ȷ��һ������ֻ����һ��
					SendCmd(CameraGrabStart,COMPUTER);							
				}				
		 }
		if((GuardMoveType == GUARDDOWNING) && (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0)) 
		{
		  HAL_TIM_OC_Stop_IT(&htim1, TIM_CHANNEL_2);
			__HAL_GPIO_EXTI_CLEAR_IT(TRIG2_Pin);							 
		 	HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_SET);
			pulse_count[0] =0;
			HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);	
			GuardMoveType = GUARDSTANDBY;
			return;
		}
			
}


/*
����CMD���� ��Ϊ���͸���λ���ͷ��͸���λ���ӻ�
*/
void SendCmd(uint8_t cmd , uint8_t direction)
{
	if(direction==COMPUTER) 
	{
		memset(RSxBuf, 0, sizeof(RSxBuf)); 	
		RSxBuf[0] = FRAME_START;
		RSxBuf[FRAME_CHECK_BEGIN] =cmd; 
		RSxBuf[FRAME_LENTH-1] = FRAME_END;			
		RSxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&RSxBuf[FRAME_CHECK_BEGIN], FRAME_CHECK_NUM);
		HAL_UART_Transmit_IT(&huart5, (uint8_t *)&RSxBuf, FRAME_LENTH);					// ����λ����Ϣ		
	}
	if(direction==SLAVE) 
	{
		memset(TSxBuf, 0, sizeof(TSxBuf)); 
		TSxBuf[0] = FRAME_START;
		TSxBuf[FRAME_CHECK_BEGIN] = cmd;  
		TSxBuf[FRAME_LENTH-1] = FRAME_END;			
		TSxBuf[FRAME_CHECKSUM] = CheckSum((uint8_t*)&TSxBuf[FRAME_CHECK_BEGIN], FRAME_CHECK_NUM);
		HAL_UART_Transmit_IT(&huart1, (uint8_t *)&TSxBuf, FRAME_LENTH);					// ����λ���ӻ�������Ϣ
	}
	return;
}


//�����ȷ�ƶ�
void AccuarteMove(uint8_t plat,uint8_t axis)
{
	  accurateH=0; accurateT=0; accurateO=0; accurateD=0; accurateP=0;
	
		accurateH=WordsRecognition(RxBuf[4]);
		accurateT=WordsRecognition(RxBuf[5]);	
		accurateO=WordsRecognition(RxBuf[6]);
		accurateD=WordsRecognition(RxBuf[7]);			
		accurateP=WordsRecognition(RxBuf[8]);														 														
		accurate_speed[plat][axis]=accurateH*100+accurateT*10+accurateO+accurateD/10.0+accurateP/100.0; 
	  return;
}


/////////////////////////////////////////////////////

void RollerSetAndStart(double rSpeed)  
{
	if(rSpeed==0.00)
	{
			RollerSpeed = 0.00;
		  return;
		}	
	else
	{
			RollerSpeed=2* (rSpeed) / 60.00;
			if(RollerSpeed < 0.00)
			{
				RollerSpeed = fabs(RollerSpeed);
				HAL_GPIO_WritePin(STEPMOTOR1_DIR_PORT, STEPMOTOR1_DIR_PIN, GPIO_PIN_SET);
			}
			else{
				HAL_GPIO_WritePin(STEPMOTOR1_DIR_PORT, STEPMOTOR1_DIR_PIN, GPIO_PIN_RESET);
			}
			RollerSpeed *= (SPR-1500)*1.00;
			if(RollerSpeed != 0.00)
			{
				float tmp_pulse = T1_FREQ / (float)RollerSpeed;
				if(tmp_pulse >= 65535.0f)
					Toggle_Pulse[0] = 65535.0f;
				else 
					Toggle_Pulse[0] = (uint16_t)tmp_pulse;
			}	
			return;
	 }
}

void GuardCtrl(uint8_t status)
{		
		if(GuardMoveType == GUARDSTANDBY)
		{
			if((status == UP) && (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==0))
			{
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);					
				HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_SET);	//����
				pulse_count[0] = guardSpr;
				GuardMoveType = GUARDUPING; 
			}
				
			if((status == DOWN) && (HAL_GPIO_ReadPin(GPIOG, GPIO_PIN_9)==1))
			{
				HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_2);	
				HAL_GPIO_WritePin(STEPMOTOR2_DIR_PORT, STEPMOTOR2_DIR_PIN, GPIO_PIN_RESET);	//����
				pulse_count[0] = 6400;
  			GuardMoveType = GUARDDOWNING;			
			}	
			TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);
		}
}



uint32_t WordsRecognition(uint8_t  word)
{
	uint32_t recognition=0;
	switch(word)
	{
		case(0x00):
		{
			recognition=0;
			break;
		}
		case(0x01):
		{
			recognition=1;
			break;
		}
		case(0x02):
		{
			recognition=2;
			break;
		}
		case(0x03):
		{
			recognition=3;
			break;
		}
		case(0x04):
		{
			recognition=4;
			break;
		}
		case(0x05):
		{
			recognition=5;
			break;
		}
		case(0x06):
		{
			recognition=6;
			break;
		}
		case(0x07):
		{
			recognition=7;
			break;
		}
		case(0x08):
		{
			recognition=8;
			break;
		}
			case(0x09):
		{
			recognition=9;
			break;
		}
	}
	return recognition;
}





void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
	uint32_t count;

	if(htim == &htim1)
	{
		count = __HAL_TIM_GET_COUNTER(&htim1);
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)  //����
		{
			if(RollerSpeed != 0.00)
			{
				uint32_t tmp;
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_ENABLE);
				tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[0]);
				__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, tmp);
			}
			else
			{
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_1, TIM_CCx_DISABLE);
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC1);
			}
		}
		
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)  //��е���
		{
			uint32_t tmp;
			tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[1]);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, tmp);
			
			if(pulse_count[0] == 0)
			{
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_DISABLE);
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC2);
				
				//����UPֹͣ
				if(GuardMoveType == GUARDBUSY_UPING)
				{
					GuardMoveType = GUARDSTANDBY;
				}
				
				//�����м�⵲������º����ϻؽ���
				if((BearingOnPosition==2) && (BeginWorking == 1))
				{				
          BearingOnPosition=0;						        			
				}
			}
			else
			{
//				if((pulse_count[0] == 800) &&(BearingOnPosition==2) &&(BeginWorking == 1))
//				{
//					SendCmd(ReloadBearing,COMPUTER);
//				}
				pulse_count[0]--;
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_2, TIM_CCx_ENABLE);
			}
		}
		
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  //�а��ȵ��
		{		
			uint32_t tmp;
			tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[2]);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, tmp);
			if(pulse_count[1] == 0)
			{
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_DISABLE);
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC3);

//				if(BPlatform_HomeStep == 3)
//				{
//					BPlatform_HomeStep = 4;
//				}   ԭ��
				
				if(BPlatform_HomeStep == 5)
				{
					BPlatform_HomeStep = 6;
				}
			}
			else
			{
				pulse_count[1]--;
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_3, TIM_CCx_ENABLE);
			}
		}
		
		else if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)  //�а�߶ȵ��
		{
			uint32_t tmp;
			tmp = STEPMOTOR_TIM_PERIOD & (count + Toggle_Pulse[3]);
			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, tmp);
			if(pulse_count[2] == 0)
			{
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_4, TIM_CCx_DISABLE);
				__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_CC4);
				
//				if(BPlatform_HomeStep == 7)
//				{
//					BPlatform_HomeStep = 8;
//				}
				if(BPlatform_HomeStep == 2)
				{
					BPlatform_HomeStep = 3;
				}
			}
			else
			{
				pulse_count[2]--;
				TIM_CCxChannelCmd(TIM1, TIM_CHANNEL_4, TIM_CCx_ENABLE);
			}
		}
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */


