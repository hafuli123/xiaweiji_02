#include "StepMotor/bsp_STEPMOTOR.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <string.h>

/* ˽�����Ͷ��� --------------------------------------------------------------*/
//�������ͨ��
typedef struct { \
	GPIO_TypeDef* Pulse_Port;
	uint16_t  Pulse_Pin ; 		// ��ʱ�������������
	uint32_t  Channel;		  	// ��ʱ��ͨ��
	uint32_t  IT_CCx ;  		// ��ʱ��ͨ���ж�ʹ��λ
	uint32_t  Flag_CCx ;    	// ��ʱ��SR�жϱ��λ
}Tim;

typedef struct { \
	IRQn_Type 		IRQn;        	// �жϱ��
	uint8_t   		Active_Level;	// �ж����ŵ�ƽ
	uint16_t  		Pin ; 	      	// ���ź�
	GPIO_TypeDef 	*Port;
}Detect_PIN;              			// ԭ��������

speedRampData srd[3][4] = { \
	{
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},   // �Ӽ������߱���
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0}
	},
	{
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0}
	},
	{
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0}
	}
};

	/* ��ʱ�������������,��Ҫ�޸�ֱ����stepmotor.hͷ�ļ��޸ļ���*/
const Tim Timer[3][4] = {
	{
		{P1_STEPMOTOR_TIM_PULSE_PORT_X, P1_STEPMOTOR_TIM_PULSE_PIN_X, P1_STEPMOTOR_TIM_CHANNEL1, P1_STEPMOTOR_TIM_IT_CC1, P1_STEPMOTOR_TIM_FLAG_CC1},	//X��
		{P1_STEPMOTOR_TIM_PULSE_PORT_Y, P1_STEPMOTOR_TIM_PULSE_PIN_Y, P1_STEPMOTOR_TIM_CHANNEL2, P1_STEPMOTOR_TIM_IT_CC2, P1_STEPMOTOR_TIM_FLAG_CC2},	//Y��
		{P1_STEPMOTOR_TIM_PULSE_PORT_Z, P1_STEPMOTOR_TIM_PULSE_PIN_Z, P1_STEPMOTOR_TIM_CHANNEL3, P1_STEPMOTOR_TIM_IT_CC3, P1_STEPMOTOR_TIM_FLAG_CC3},	//Z��
		{P1_STEPMOTOR_TIM_PULSE_PORT_R, P1_STEPMOTOR_TIM_PULSE_PIN_R, P1_STEPMOTOR_TIM_CHANNEL4, P1_STEPMOTOR_TIM_IT_CC4, P1_STEPMOTOR_TIM_FLAG_CC4},	//R��
	},
	{
		{P2_STEPMOTOR_TIM_PULSE_PORT_X, P2_STEPMOTOR_TIM_PULSE_PIN_X, P2_STEPMOTOR_TIM_CHANNEL1, P2_STEPMOTOR_TIM_IT_CC1, P2_STEPMOTOR_TIM_FLAG_CC1},	//X��
		{P2_STEPMOTOR_TIM_PULSE_PORT_Y, P2_STEPMOTOR_TIM_PULSE_PIN_Y, P2_STEPMOTOR_TIM_CHANNEL2, P2_STEPMOTOR_TIM_IT_CC2, P2_STEPMOTOR_TIM_FLAG_CC2},	//Y��
		{P2_STEPMOTOR_TIM_PULSE_PORT_Z, P2_STEPMOTOR_TIM_PULSE_PIN_Z, P2_STEPMOTOR_TIM_CHANNEL3, P2_STEPMOTOR_TIM_IT_CC3, P2_STEPMOTOR_TIM_FLAG_CC3},	//Z��
		{P2_STEPMOTOR_TIM_PULSE_PORT_R, P2_STEPMOTOR_TIM_PULSE_PIN_R, P2_STEPMOTOR_TIM_CHANNEL4, P2_STEPMOTOR_TIM_IT_CC4, P2_STEPMOTOR_TIM_FLAG_CC4},	//R��
	},
	{
		{P3_STEPMOTOR_TIM_PULSE_PORT_X, P3_STEPMOTOR_TIM_PULSE_PIN_X, P3_STEPMOTOR_TIM_CHANNEL1, P3_STEPMOTOR_TIM_IT_CC1, P3_STEPMOTOR_TIM_FLAG_CC1},	//X��
		{P3_STEPMOTOR_TIM_PULSE_PORT_Y, P3_STEPMOTOR_TIM_PULSE_PIN_Y, P3_STEPMOTOR_TIM_CHANNEL2, P3_STEPMOTOR_TIM_IT_CC2, P3_STEPMOTOR_TIM_FLAG_CC2},	//Y��
		{P3_STEPMOTOR_TIM_PULSE_PORT_Z, P3_STEPMOTOR_TIM_PULSE_PIN_Z, P3_STEPMOTOR_TIM_CHANNEL3, P3_STEPMOTOR_TIM_IT_CC3, P3_STEPMOTOR_TIM_FLAG_CC3},	//Z��
		{P3_STEPMOTOR_TIM_PULSE_PORT_R, P3_STEPMOTOR_TIM_PULSE_PIN_R, P3_STEPMOTOR_TIM_CHANNEL4, P3_STEPMOTOR_TIM_IT_CC4, P3_STEPMOTOR_TIM_FLAG_CC4},	//R��
	}
};


	/* ���������������*/
const StepMotor Stepmotor[3][4] = {
	{
		{P1_STEPMOTOR_X_ENA_PIN, P1_STEPMOTOR_X_DIR_PIN, P1_STEPMOTOR_X_DIR_PORT, P1_STEPMOTOR_X_ENA_PORT},
		{P1_STEPMOTOR_Y_ENA_PIN, P1_STEPMOTOR_Y_DIR_PIN, P1_STEPMOTOR_Y_DIR_PORT, P1_STEPMOTOR_Y_ENA_PORT},
		{P1_STEPMOTOR_Z_ENA_PIN, P1_STEPMOTOR_Z_DIR_PIN, P1_STEPMOTOR_Z_DIR_PORT, P1_STEPMOTOR_Z_ENA_PORT},
		{P1_STEPMOTOR_R_ENA_PIN, P1_STEPMOTOR_R_DIR_PIN, P1_STEPMOTOR_R_DIR_PORT, P1_STEPMOTOR_R_ENA_PORT}
	},
	{
		{P2_STEPMOTOR_X_ENA_PIN, P2_STEPMOTOR_X_DIR_PIN, P2_STEPMOTOR_X_DIR_PORT, P2_STEPMOTOR_X_ENA_PORT},
		{P2_STEPMOTOR_Y_ENA_PIN, P2_STEPMOTOR_Y_DIR_PIN, P2_STEPMOTOR_Y_DIR_PORT, P2_STEPMOTOR_Y_ENA_PORT},
		{P2_STEPMOTOR_Z_ENA_PIN, P2_STEPMOTOR_Z_DIR_PIN, P2_STEPMOTOR_Z_DIR_PORT, P2_STEPMOTOR_Z_ENA_PORT},
		{P2_STEPMOTOR_R_ENA_PIN, P2_STEPMOTOR_R_DIR_PIN, P2_STEPMOTOR_R_DIR_PORT, P2_STEPMOTOR_R_ENA_PORT}
	},
	{
		{P3_STEPMOTOR_X_ENA_PIN, P3_STEPMOTOR_X_DIR_PIN, P3_STEPMOTOR_X_DIR_PORT, P3_STEPMOTOR_X_ENA_PORT},
		{P3_STEPMOTOR_Y_ENA_PIN, P3_STEPMOTOR_Y_DIR_PIN, P3_STEPMOTOR_Y_DIR_PORT, P3_STEPMOTOR_Y_ENA_PORT},
		{P3_STEPMOTOR_Z_ENA_PIN, P3_STEPMOTOR_Z_DIR_PIN, P3_STEPMOTOR_Z_DIR_PORT, P3_STEPMOTOR_Z_ENA_PORT},
		{P3_STEPMOTOR_R_ENA_PIN, P3_STEPMOTOR_R_DIR_PIN, P3_STEPMOTOR_R_DIR_PORT, P3_STEPMOTOR_R_ENA_PORT}
	}
};
	/* ��ת��λ���״̬*/
__IO uint8_t  LimPosi[3][4] = {
	{FALSE, FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE}
};

	/* ��ת��λ���״̬*/
__IO uint8_t  LimNega[3][4] = {
	{FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE}
};
	/* �������״̬*/
__IO uint8_t MotionStatus[3][4] = {
	{STOP, STOP, STOP, STOP},
	{STOP, STOP, STOP, STOP},
	{STOP, STOP, STOP, STOP}
};
	/* ����˶�����*/
__IO int8_t OriginDir[3][4] = {
	{MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW},
	{MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW},
	{MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW}
};

	/* �����ǰλ�õ��ۼ�������*/
__IO int32_t step_position[3][4] = {
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

	/* �����ǰλ�õľ���*/
__IO uint16_t location[3][4] = {
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

	/* ԭ����*/
__IO uint8_t OriginFlag[3][4] = {
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

	/* �����ʼλ��״̬*/
__IO  BeginState_Typedef  BeginState[3][4] = {
	{NORMAL, NORMAL, NORMAL, NORMAL},
	{NORMAL, NORMAL, NORMAL, NORMAL},
	{NORMAL, NORMAL, NORMAL, NORMAL}
};

	/* ���ƽ̨1��2��3���Ƶ���Ķ�ʱ������*/
TIM_TypeDef * STEPMOTOR_TIMx[3] = {TIM8, TIM3, TIM4};

/* ���ڼ�����ʵ�Ԥ��Ƶϵ������ʱ���� */
uint16_t Prescaler = STEPMOTOR_TIM_PRESCALER;	// Ĭ�϶�ʱ��Ԥ��Ƶ
float A_T_x10 = 0;                 
float SM_LSC_TIMx_Freq = 0;         			// ��ʱ��Ƶ��
float SM_LSC_TIMx_Freq_148 = 0;     			// ��ʱ��Ƶ�ʳ��ϳ�������֮��ı���

/* ��չ���� ------------------------------------------------------------------*/
extern uint32_t set_speed;          // �ٶ� ��λΪ0.05rad/sec
extern uint32_t step_accel;         // ���ٶ� ��λΪ0.025rad/sec^2
extern uint32_t step_decel;         // ���ٶ� ��λΪ0.025rad/sec^2
extern uint32_t fastseek_Speed ;    // �ع��ٶ�
extern uint32_t slowseek_Speed ;    // �����ٶ�


extern __IO uint32_t pulse_count[4];
extern __IO uint8_t BeginWorking;

extern __IO uint8_t BearingPlatform_AdjustDone;
extern __IO uint8_t CameraMotor_AdjustType;
extern __IO uint32_t CameraMotor_AdjustDone[4][4];

 extern __IO uint32_t PreAdjusting_Work;
 extern __IO uint32_t PreAdjust_Step;
 extern __IO uint32_t PreAdjust_Done[4][4];



/**
  * ��������: ���λ���˶����˶������Ĳ���
  * �������: Axis   ���
  *           step   �ƶ��Ĳ��� (����Ϊ˳ʱ�룬����Ϊ��ʱ��).
  *           accel  ���ٶ�,ʵ��ֵΪaccel*0.1*rad/sec^2
  *           decel  ���ٶ�,ʵ��ֵΪdecel*0.1*rad/sec^2
  *           speed  ����ٶ�,ʵ��ֵΪspeed*0.025*rad/sec
  * �� �� ֵ: ��
  * ˵    ��: �Ը����Ĳ����ƶ�����������ȼ��ٵ�����ٶȣ�Ȼ���ں���λ�ÿ�ʼ
  *           ������ֹͣ��ʹ�������˶�����Ϊָ���Ĳ���������Ӽ��ٽ׶κ̲ܶ���
  *           �ٶȺ������ǻ�û�ﵽ����ٶȾ�Ҫ��ʼ����
  */
void STEPMOTOR_AxisMoveRel(uint8_t Plat, uint8_t Axis, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	__IO uint16_t tim_count;	//��ȡ��ʱ���ļ���ֵ
	// �ﵽ����ٶ�ʱ�Ĳ���
	__IO uint32_t max_s_lim;
	// ����Ҫ��ʼ���ٵĲ������������û�дﵽ����ٶȣ�
	__IO uint32_t accel_lim;
  
	//�ڵ���˶�����һ�㲻���ټ�������,�����ڻ�ԭ�������
	if(MotionStatus[Plat][Axis] != STOP)	
	{
		return;
	}
	memset((void*)&srd[Plat][Axis], 0, sizeof(srd[Plat][Axis]));
  
	/* �����趨 */
	if(step < 0) // ����Ϊ����
	{
		srd[Plat][Axis].dir = MOTOR_DIR_CCW; 	// ��ʱ�뷽����ת
		STEPMOTOR_DIR_REVERSAL(Plat, Axis);
		step = -step;   				// ��ȡ��������ֵ
	}
	else
	{
		srd[Plat][Axis].dir = MOTOR_DIR_CW; 	// ˳ʱ�뷽����ת
		STEPMOTOR_DIR_FORWARD(Plat, Axis);
	}

	if(step == 1)    // ����Ϊ1
	{
		srd[Plat][Axis].accel_count 	= -1;   	// ֻ�ƶ�һ��
		srd[Plat][Axis].run_state 	= DECEL;  	// ����״̬.
		srd[Plat][Axis].step_delay 	= 1000;		// ����ʱ	
	}
	else if(step == 0)
	{
		return;
	}
	else if(step != 0)  // ���Ŀ���˶�������Ϊ0
	{
		// �ڵ������ר��ָ���ֲ�����ϸ�ļ��㼰�Ƶ�����

		/** ʹ��doѭ����ȷ�����ʵĶ�ʱ��Ԥ��Ƶ,ȷ��������ʱ���ٶ��㹻��,��
		 *  Ԥ��ƵԽ��,����Դﵽ������ٶ���Խ��.
		 */
		do
		{ 	// ��һ���ǵ�����ʱ��Ԥ��Ƶ,ʹ�������������
			__HAL_TIM_SET_PRESCALER(htimx[Plat], Prescaler++);
			SM_LSC_TIMx_Freq 		= ((float)SystemCoreClock / Prescaler);      
			SM_LSC_TIMx_Freq_148 	= (((SM_LSC_TIMx_Freq * 0.676f) / 10.0f));
		  
			// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2

			srd[Plat][Axis].step_delay = round((SM_LSC_TIMx_Freq_148 * sqrt(A_SQ / accel)) / 10);//C0,��ʼ�ٶȵĶ�ʱ��ֵ
				
		} while(srd[Plat][Axis].step_delay > (uint32_t)0xFFFF);
		
		A_T_x10 = ((float)(10*ALPHA*SM_LSC_TIMx_Freq));
		// ��������ٶȼ���, ����õ�min_delay���ڶ�ʱ���ļ�������ֵ��

		srd[Plat][Axis].min_delay = (int32_t)(A_T_x10 / speed);

		// ͨ�������һ��(c0) �Ĳ�����ʱ���趨���ٶȣ�����accel��λΪ0.1rad/sec^2

		srd[Plat][Axis].step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10);//C0,��ʼ�ٶȵĶ�ʱ��ֵ

		/*����Ӽ�����Ҫ�Ĳ���*/
		// ������ٲ�֮��ﵽ����ٶȵ�����
		max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel / 10));
		// ����ﵽ����ٶ�С��0.5�������ǽ���������Ϊ0
		// ��ʵ�����Ǳ����ƶ�����һ�����ܴﵽ��Ҫ���ٶ�
		if(max_s_lim == 0)
		{
			max_s_lim = 1;
		}
			
		// ������ٲ�֮�����Ǳ��뿪ʼ����

		accel_lim = (uint32_t)(step * decel / (accel + decel));
		// ���Ǳ����������1�����ܲ��ܿ�ʼ����.
		if(accel_lim == 0)
		{
			accel_lim = 1;
		}
		// ʹ�������������ǿ��Լ�������ٽ׶β���
		if(accel_lim <= max_s_lim)
		{
			srd[Plat][Axis].decel_val = accel_lim - step;
		}
		else
		{
			srd[Plat][Axis].decel_val = -(max_s_lim * accel / decel);
		}
		// ��ֻʣ��һ�����Ǳ������
		if(srd[Plat][Axis].decel_val == 0)
		{
			srd[Plat][Axis].decel_val = -1;
		}

		// ���㿪ʼ����ʱ�Ĳ���
		srd[Plat][Axis].decel_start = step + srd[Plat][Axis].decel_val;

		// �������ٶȺ��������ǾͲ���Ҫ���м����˶�
		if(srd[Plat][Axis].step_delay <= srd[Plat][Axis].min_delay)
		{
			srd[Plat][Axis].step_delay 	= srd[Plat][Axis].min_delay;
			srd[Plat][Axis].run_state 	= RUN;
		}
		else
		{
			srd[Plat][Axis].run_state = ACCEL;
		}    
		// ��λ���ٶȼ���ֵ
		srd[Plat][Axis].accel_count = 0;
		
	}
	MotionStatus[Plat][Axis] = 1;
  
	tim_count = __HAL_TIM_GET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel);
	tim_count += (srd[Plat][Axis].step_delay >> 1) ;
	__HAL_TIM_SET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel, (uint16_t)tim_count); // ���ö�ʱ���Ƚ�ֵ
	HAL_TIM_OC_Start_IT(htimx[Plat], Timer[Plat][Axis].Channel);
	STEPMOTOR_OUTPUT_ENABLE(Plat, Axis);
}


/** ��������: �����ƶ�
  * �������: targert_step:Ŀ���λ��
  *			      accel:���ٶ�
  *			      decel:���ٶ�
  *			      speed:����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ֶ����ƶ�,����Ŀ��λ�������ԭ��Ĳ���,
  *  		      �Լ��ٶȼ��ٵ�����ٶȺ�����Ѱ��Ŀ��λ��,
  *			      Ȼ���ں��ʵ�λ�ü���,����Ŀ��λ��.
  */
void STEPMOTOR_AxisMoveAbs(uint8_t Plat, uint8_t Axis, int32_t targert_step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	int32_t rel_step = 0;
	int8_t dir = -1;                        //��ԭ��ķ������
	
	rel_step = step_position[Plat][Axis] - targert_step ; 	//��ȡ��ǰλ�ú�Ŀ��λ��֮��Ĳ���ֵ
	
	if(rel_step == 0)	
	{
		dir = 0;
	}
	STEPMOTOR_AxisMoveRel(Plat, Axis, dir * rel_step, accel, decel, speed);
	
	return;
}


/** ��������: ����ƶ�һ������;��λ:mm
  * �������: distance  ��Ҫ�ƶ��ľ���,���������ַ���,��������������,
  *                    �������Ƿ�����
  *            accel  ���ٶ�
  *            decel  ���ٶ�
  *            speed  ����ٶ�
  * �� �� ֵ: ��
  * ˵    ��: �ӵ�ǰλ�����ڵ������ƶ�һ���ľ���,�Բ���distance���������ַ���
 */
void STEPMOTOR_DisMoveRel(__IO uint8_t Plat, __IO uint8_t Axis, __IO int16_t distance, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
	__IO int32_t step;
	step = distance * UNIT_STEP_MM;//��ò���Ŀ�����Ĳ���
	
	STEPMOTOR_AxisMoveRel(Plat, Axis, step, accel, decel, speed);
	
	return;
}

/**
  * ��������: �ƶ���Ŀ��λ��;��λ:mm
  * �������: Target_Dis  Ŀ������λ��
  *           accel  ���ٶ�
  *           decel  ���ٶ�
  *           speed  ����ٶ�
  * �� �� ֵ: ����ٶ�
  * ˵    ��: �ƶ�������������λ��,���ݵ�ǰ��λ�ü������Ŀ��λ�õľ���,
  *           ���ƶ���Ŀ��λ��,
  *             
  */
void STEPMOTOR_DisMoveAbs(uint8_t Plat, uint8_t Axis, uint16_t Target_Dis, uint32_t accel, uint32_t decel, uint32_t speed)
{
	int32_t step; 
    
	if(Target_Dis > MAX_DISTANCE)
		return;
	else if(Target_Dis == 0) // < UNITS_DISTANCE)
		return;
	
	step = (Target_Dis - location[Plat][Axis]) * UNIT_STEP_MM;
	STEPMOTOR_AxisMoveRel(Plat, Axis, step, accel, decel, speed);
	
	return;
}

void HAL_TIMx_OC_Callback(uint8_t Plat, uint8_t Axis)
{
	__IO static uint16_t tim_count 				= 0;
	__IO uint32_t new_step_delay 				= 0;
	__IO static uint8_t  i[4]					= {0, 0, 0, 0};
	__IO static uint16_t last_accel_delay[4]	= {0, 0, 0, 0};
	// ���ƶ�����������
	__IO static uint32_t step_count[4]			= {0, 0, 0, 0};
	// ��¼new_step_delay�е������������һ������ľ���
	__IO static int32_t rest[4]					= {0, 0, 0, 0};
	//��ʱ��ʹ�÷�תģʽ����Ҫ���������жϲ����һ����������

	// ���ñȽ�ֵ
	tim_count = __HAL_TIM_GET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel);
	tim_count += (srd[Plat][Axis].step_delay >> 1);
	__HAL_TIM_SET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel, (uint16_t)tim_count);
	
	i[Axis]++;       // ��ʱ���жϴ�������ֵ
	if(i[Axis] == 2)
	{
		i[Axis] = 0;
		
		/* �ж��Ƿ��ڱ�Ե�˶� */
		if(Axis != AXIS_R)
		{
			/* �ж��Ƿ��ڱ�Ե�˶� */
			if(srd[Plat][Axis].dir == MOTOR_DIR_CCW)
			{
				if(LimNega[Plat][Axis] == TRUE)		//��ʱ���� ������ת�ļ���λ��,ֹͣ�˶�
				{
					srd[Plat][Axis].run_state 	= STOP;
					MotionStatus[Plat][Axis] 	= STOP;
					LimPosi[Plat][Axis] 		= FALSE;//��ת�ļ��ޱ�־λӦΪFALSE
				}
				else 
				{
					LimPosi[Plat][Axis] = FALSE;	  	//����ʱ�뷽��,��������ת�ļ���λ��֮ǰ,��ת�ļ��ޱ�־λӦΪFALSE
				}
			}
			else 
			{
				if(LimPosi[Plat][Axis] == TRUE)		
				{
					srd[Plat][Axis].run_state 	= STOP;
					MotionStatus[Plat][Axis] 	= STOP;
					LimNega[Plat][Axis] 		= FALSE;//��˳ʱ�뷽��,��������λ��,��ת���ޱ�־λӦΪFalse
				}
				else 
				{
					LimNega[Plat][Axis] = FALSE;  	//��˳ʱ�뷽��,��������ת�ļ���λ��֮ǰ,��ת�ļ��ޱ�־λӦΪFALSE
				}
			}
		}
		switch(srd[Plat][Axis].run_state) 		// �Ӽ������߽׶�
		{
			case STOP:
			{
				// �ر�ͨ��
				TIM_CCxChannelCmd(STEPMOTOR_TIMx[Plat], Timer[Plat][Axis].Channel, TIM_CCx_DISABLE);       
				__HAL_TIM_CLEAR_FLAG(htimx[Plat], Timer[Plat][Axis].Flag_CCx);
				HAL_TIM_OC_Stop_IT(htimx[Plat], Timer[Plat][Axis].Channel);
			  
				MotionStatus[Plat][Axis] 		= STOP;  	//  ���Ϊֹͣ״̬
				step_count[Axis] 			= 0;  		// ���㲽��������
				rest[Axis]					= 0;        // ������ֵ
				last_accel_delay[Axis]		= 0;
				srd[Plat][Axis].accel_count 	= 0;
				srd[Plat][Axis].step_delay 	= 0;
				srd[Plat][Axis].min_delay 		= 0;
			
				if( PreAdjusting_Work == 1)
				{
					PreAdjust_Done[Plat][Axis] = 1;
				}
				
				
				
				//���ƽ̨������λ��ʱ ����P3R���˵��
				if((Plat == P3)&&(Axis == AXIS_R))
				{
					if(BearingPlatform_AdjustDone == 1)
					{
						BearingPlatform_AdjustDone = 2;						
					}
					break;
				}
				
				//���ƽ̨���������λ���
				if(CameraMotor_AdjustType!=0)
				{
					CameraMotor_AdjustDone[Plat][Axis] = 1;
				}
							
				break;
			}
			case ACCEL:
			{
				step_count[Axis]++;      			// ������1
				if(srd[Plat][Axis].dir == MOTOR_DIR_CW)
				{		  	
					step_position[Plat][Axis]++; 	  		// ����λ�ü�1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]++;
				}
				else
				{
					step_position[Plat][Axis]--; 	  		// ����λ�ü�1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]--;
				}
				srd[Plat][Axis].accel_count++; 	  		// ���ټ���ֵ��1
				
				//������(��)һ����������(ʱ����)
				new_step_delay 	= srd[Plat][Axis].step_delay - (((2 * srd[Plat][Axis].step_delay) + rest[Axis]) / (4 * srd[Plat][Axis].accel_count + 1));	
				// �����������´μ��㲹���������������
				rest[Axis] = ((2 * srd[Plat][Axis].step_delay) + rest[Axis]) % (4 * srd[Plat][Axis].accel_count + 1);							
				
				if(step_count[Axis] >= srd[Plat][Axis].decel_start)			// ����Ƿ�Ӧ�ÿ�ʼ����
				{
					srd[Plat][Axis].accel_count 	= srd[Plat][Axis].decel_val; 		// ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
					srd[Plat][Axis].run_state 		= DECEL;           	      	// �¸����������ٽ׶�
				}
				else if(new_step_delay <= srd[Plat][Axis].min_delay)  		// ����Ƿ񵽴�����������ٶ�
				{
					srd[Plat][Axis].accel_count 	= srd[Plat][Axis].decel_val; 		// ���ټ���ֵΪ���ٽ׶μ���ֵ�ĳ�ʼֵ
					last_accel_delay[Axis] 		= new_step_delay; 			// ������ٹ��������һ����ʱ���������ڣ�
					new_step_delay 				= srd[Plat][Axis].min_delay;    	// ʹ��min_delay����Ӧ����ٶ�speed��
					rest[Axis]					= 0;                          	// ������ֵ
					srd[Plat][Axis].run_state 		= RUN;               		// ����Ϊ��������״̬
				}	
				last_accel_delay[Axis] = new_step_delay; 	  			// ������ٹ��������һ����ʱ���������ڣ�
				break;
			}
			case RUN:
			{
				step_count[Axis]++; 		 		// ������1
				if(srd[Plat][Axis].dir == MOTOR_DIR_CW)
				{	  	
					step_position[Plat][Axis]++; 			// ����λ�ü�1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]++;
				}
				else
				{
					step_position[Plat][Axis]--; 			// ����λ�ü�1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]--;
				}
				
				new_step_delay = srd[Plat][Axis].min_delay;       			// ʹ��min_delay����Ӧ����ٶ�speed��
					
				if(step_count[Axis] >= srd[Plat][Axis].decel_start)   		// ��Ҫ��ʼ����
				{
					srd[Plat][Axis].accel_count 	= srd[Plat][Axis].decel_val;  	// ���ٲ�����Ϊ���ټ���ֵ
					new_step_delay				= last_accel_delay[Axis];	// �ӽ׶�������ʱ��Ϊ���ٽ׶ε���ʼ��ʱ(��������)
					srd[Plat][Axis].run_state 		= DECEL;            		// ״̬�ı�Ϊ����
				}
				break;
			}
			case DECEL:
			{
				step_count[Axis]++; 		 		// ������1
				if(srd[Plat][Axis].dir == MOTOR_DIR_CW)
				{		  	
					step_position[Plat][Axis]++; 			// ����λ�ü�1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]++;
				}
				else
				{
					step_position[Plat][Axis]--; 			// ����λ�ü�1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]--;
				}
				srd[Plat][Axis].accel_count++;
				new_step_delay = srd[Plat][Axis].step_delay - (((2 * srd[Plat][Axis].step_delay) + rest[Axis]) / (4 * srd[Plat][Axis].accel_count + 1)); 	//������(��)һ����������(ʱ����)
				rest[Axis] = ((2 * srd[Plat][Axis].step_delay) + rest[Axis]) % (4 * srd[Plat][Axis].accel_count + 1);								// �����������´μ��㲹���������������
				
				//����Ƿ�Ϊ���һ��
				if(srd[Plat][Axis].accel_count >= 0)
				{
					StopMotor(Plat, Axis);
				}
				break;
			}
			case DECEL_MEDLE:
			{
				new_step_delay = srd[Plat][Axis].step_delay - (((2 * srd[Plat][Axis].step_delay) + rest[Axis])/(4 * srd[Plat][Axis].accel_count + 1)); //������(��)һ����������(ʱ����)
				rest[Axis] = ((2 * srd[Plat][Axis].step_delay) + rest[Axis]) % (4 * srd[Plat][Axis].accel_count + 1);// �����������´μ��㲹���������������
				//����Ƿ�Ϊ���һ��
				if(new_step_delay >= srd[Plat][Axis].medle_delay )
				{
					srd[Plat][Axis].min_delay 	= srd[Plat][Axis].medle_delay;
					step_count[Axis] 						= 0;  		// ���㲽��������
					rest[Axis] 							= 0;        // ������ֵ
					srd[Plat][Axis].run_state 	= RUN;              
				}
				break;
			}
			default:
				break;
		}
		if( (new_step_delay >> 1) > 0xFFFF)
			new_step_delay 			= 0x1FFFF;
			srd[Plat][Axis].step_delay = new_step_delay; // Ϊ�¸�(�µ�)��ʱ(��������)��ֵ
	}
	
	return;
}


/**
  * ��������: ֹͣ���ת��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ֹͣ��ʱ���жϺ��������������Ӽ��ٵĲ�����
  *             
  */
void StopMotor(uint8_t Plat, uint8_t Axis)
{
	TIM_CCxChannelCmd(STEPMOTOR_TIMx[Plat], Timer[Plat][Axis].Channel, TIM_CCx_DISABLE);       
	srd[Plat][Axis].accel_count 	= 0;
	srd[Plat][Axis].step_delay  	= 0;
	srd[Plat][Axis].min_delay   	= 0;
	srd[Plat][Axis].medle_delay 	= 0;
	srd[Plat][Axis].run_state 		= STOP;
	MotionStatus[Plat][Axis]  		= STOP;	
}

//�سǲ���

void ResponsePoint(uint8_t Plat , uint8_t Axis)
{
		StopMotor(Plat, Axis);
		step_position[Plat][Axis]=0;
		location[Plat][Axis]=0;
		memset((void*)&srd[Plat][Axis], 0, sizeof(srd[Plat][Axis]));		
    return;	
}


