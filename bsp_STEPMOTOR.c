#include "StepMotor/bsp_STEPMOTOR.h"
#include "tim.h"
#include "gpio.h"
#include <math.h>
#include <string.h>

/* 私有类型定义 --------------------------------------------------------------*/
//脉冲输出通道
typedef struct { \
	GPIO_TypeDef* Pulse_Port;
	uint16_t  Pulse_Pin ; 		// 定时器脉冲输出引脚
	uint32_t  Channel;		  	// 定时器通道
	uint32_t  IT_CCx ;  		// 定时器通道中断使能位
	uint32_t  Flag_CCx ;    	// 定时器SR中断标记位
}Tim;

typedef struct { \
	IRQn_Type 		IRQn;        	// 中断编号
	uint8_t   		Active_Level;	// 中断引脚电平
	uint16_t  		Pin ; 	      	// 引脚号
	GPIO_TypeDef 	*Port;
}Detect_PIN;              			// 原点检测引脚

speedRampData srd[3][4] = { \
	{
		{STOP, MOTOR_DIR_CCW, 0, 0, 0, 0, 0},   // 加减速曲线变量
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

	/* 定时器脉冲输出引脚,需要修改直接在stepmotor.h头文件修改即可*/
const Tim Timer[3][4] = {
	{
		{P1_STEPMOTOR_TIM_PULSE_PORT_X, P1_STEPMOTOR_TIM_PULSE_PIN_X, P1_STEPMOTOR_TIM_CHANNEL1, P1_STEPMOTOR_TIM_IT_CC1, P1_STEPMOTOR_TIM_FLAG_CC1},	//X轴
		{P1_STEPMOTOR_TIM_PULSE_PORT_Y, P1_STEPMOTOR_TIM_PULSE_PIN_Y, P1_STEPMOTOR_TIM_CHANNEL2, P1_STEPMOTOR_TIM_IT_CC2, P1_STEPMOTOR_TIM_FLAG_CC2},	//Y轴
		{P1_STEPMOTOR_TIM_PULSE_PORT_Z, P1_STEPMOTOR_TIM_PULSE_PIN_Z, P1_STEPMOTOR_TIM_CHANNEL3, P1_STEPMOTOR_TIM_IT_CC3, P1_STEPMOTOR_TIM_FLAG_CC3},	//Z轴
		{P1_STEPMOTOR_TIM_PULSE_PORT_R, P1_STEPMOTOR_TIM_PULSE_PIN_R, P1_STEPMOTOR_TIM_CHANNEL4, P1_STEPMOTOR_TIM_IT_CC4, P1_STEPMOTOR_TIM_FLAG_CC4},	//R轴
	},
	{
		{P2_STEPMOTOR_TIM_PULSE_PORT_X, P2_STEPMOTOR_TIM_PULSE_PIN_X, P2_STEPMOTOR_TIM_CHANNEL1, P2_STEPMOTOR_TIM_IT_CC1, P2_STEPMOTOR_TIM_FLAG_CC1},	//X轴
		{P2_STEPMOTOR_TIM_PULSE_PORT_Y, P2_STEPMOTOR_TIM_PULSE_PIN_Y, P2_STEPMOTOR_TIM_CHANNEL2, P2_STEPMOTOR_TIM_IT_CC2, P2_STEPMOTOR_TIM_FLAG_CC2},	//Y轴
		{P2_STEPMOTOR_TIM_PULSE_PORT_Z, P2_STEPMOTOR_TIM_PULSE_PIN_Z, P2_STEPMOTOR_TIM_CHANNEL3, P2_STEPMOTOR_TIM_IT_CC3, P2_STEPMOTOR_TIM_FLAG_CC3},	//Z轴
		{P2_STEPMOTOR_TIM_PULSE_PORT_R, P2_STEPMOTOR_TIM_PULSE_PIN_R, P2_STEPMOTOR_TIM_CHANNEL4, P2_STEPMOTOR_TIM_IT_CC4, P2_STEPMOTOR_TIM_FLAG_CC4},	//R轴
	},
	{
		{P3_STEPMOTOR_TIM_PULSE_PORT_X, P3_STEPMOTOR_TIM_PULSE_PIN_X, P3_STEPMOTOR_TIM_CHANNEL1, P3_STEPMOTOR_TIM_IT_CC1, P3_STEPMOTOR_TIM_FLAG_CC1},	//X轴
		{P3_STEPMOTOR_TIM_PULSE_PORT_Y, P3_STEPMOTOR_TIM_PULSE_PIN_Y, P3_STEPMOTOR_TIM_CHANNEL2, P3_STEPMOTOR_TIM_IT_CC2, P3_STEPMOTOR_TIM_FLAG_CC2},	//Y轴
		{P3_STEPMOTOR_TIM_PULSE_PORT_Z, P3_STEPMOTOR_TIM_PULSE_PIN_Z, P3_STEPMOTOR_TIM_CHANNEL3, P3_STEPMOTOR_TIM_IT_CC3, P3_STEPMOTOR_TIM_FLAG_CC3},	//Z轴
		{P3_STEPMOTOR_TIM_PULSE_PORT_R, P3_STEPMOTOR_TIM_PULSE_PIN_R, P3_STEPMOTOR_TIM_CHANNEL4, P3_STEPMOTOR_TIM_IT_CC4, P3_STEPMOTOR_TIM_FLAG_CC4},	//R轴
	}
};


	/* 步进电机控制引脚*/
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
	/* 正转限位检测状态*/
__IO uint8_t  LimPosi[3][4] = {
	{FALSE, FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE}
};

	/* 反转限位检测状态*/
__IO uint8_t  LimNega[3][4] = {
	{FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE},
	{FALSE, FALSE, FALSE, FALSE}
};
	/* 电机运行状态*/
__IO uint8_t MotionStatus[3][4] = {
	{STOP, STOP, STOP, STOP},
	{STOP, STOP, STOP, STOP},
	{STOP, STOP, STOP, STOP}
};
	/* 电机运动方向*/
__IO int8_t OriginDir[3][4] = {
	{MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW},
	{MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW},
	{MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW, MOTOR_DIR_CCW}
};

	/* 电机当前位置的累计脉冲数*/
__IO int32_t step_position[3][4] = {
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

	/* 电机当前位置的距离*/
__IO uint16_t location[3][4] = {
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

	/* 原点标记*/
__IO uint8_t OriginFlag[3][4] = {
	{0, 0, 0, 0},
	{0, 0, 0, 0},
	{0, 0, 0, 0}
};

	/* 电机起始位置状态*/
__IO  BeginState_Typedef  BeginState[3][4] = {
	{NORMAL, NORMAL, NORMAL, NORMAL},
	{NORMAL, NORMAL, NORMAL, NORMAL},
	{NORMAL, NORMAL, NORMAL, NORMAL}
};

	/* 相机平台1、2、3控制电机的定时器变量*/
TIM_TypeDef * STEPMOTOR_TIMx[3] = {TIM8, TIM3, TIM4};

/* 用于计算合适的预分频系数的临时变量 */
uint16_t Prescaler = STEPMOTOR_TIM_PRESCALER;	// 默认定时器预分频
float A_T_x10 = 0;                 
float SM_LSC_TIMx_Freq = 0;         			// 定时器频率
float SM_LSC_TIMx_Freq_148 = 0;     			// 定时器频率乘上常数参数之后的变量

/* 扩展变量 ------------------------------------------------------------------*/
extern uint32_t set_speed;          // 速度 单位为0.05rad/sec
extern uint32_t step_accel;         // 加速度 单位为0.025rad/sec^2
extern uint32_t step_decel;         // 减速度 单位为0.025rad/sec^2
extern uint32_t fastseek_Speed ;    // 回归速度
extern uint32_t slowseek_Speed ;    // 爬行速度


extern __IO uint32_t pulse_count[4];
extern __IO uint8_t BeginWorking;

extern __IO uint8_t BearingPlatform_AdjustDone;
extern __IO uint8_t CameraMotor_AdjustType;
extern __IO uint32_t CameraMotor_AdjustDone[4][4];

 extern __IO uint32_t PreAdjusting_Work;
 extern __IO uint32_t PreAdjust_Step;
 extern __IO uint32_t PreAdjust_Done[4][4];



/**
  * 函数功能: 相对位置运动：运动给定的步数
  * 输入参数: Axis   活动轴
  *           step   移动的步数 (正数为顺时针，负数为逆时针).
  *           accel  加速度,实际值为accel*0.1*rad/sec^2
  *           decel  减速度,实际值为decel*0.1*rad/sec^2
  *           speed  最大速度,实际值为speed*0.025*rad/sec
  * 返 回 值: 无
  * 说    明: 以给定的步数移动步进电机，先加速到最大速度，然后在合适位置开始
  *           减速至停止，使得整个运动距离为指定的步数。如果加减速阶段很短并且
  *           速度很慢，那还没达到最大速度就要开始减速
  */
void STEPMOTOR_AxisMoveRel(uint8_t Plat, uint8_t Axis, int32_t step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	__IO uint16_t tim_count;	//获取定时器的计数值
	// 达到最大速度时的步数
	__IO uint32_t max_s_lim;
	// 必须要开始减速的步数（如果加速没有达到最大速度）
	__IO uint32_t accel_lim;
  
	//在电机运动过程一般不能再继续操作,除了在回原点过程中
	if(MotionStatus[Plat][Axis] != STOP)	
	{
		return;
	}
	memset((void*)&srd[Plat][Axis], 0, sizeof(srd[Plat][Axis]));
  
	/* 方向设定 */
	if(step < 0) // 步数为负数
	{
		srd[Plat][Axis].dir = MOTOR_DIR_CCW; 	// 逆时针方向旋转
		STEPMOTOR_DIR_REVERSAL(Plat, Axis);
		step = -step;   				// 获取步数绝对值
	}
	else
	{
		srd[Plat][Axis].dir = MOTOR_DIR_CW; 	// 顺时针方向旋转
		STEPMOTOR_DIR_FORWARD(Plat, Axis);
	}

	if(step == 1)    // 步数为1
	{
		srd[Plat][Axis].accel_count 	= -1;   	// 只移动一步
		srd[Plat][Axis].run_state 	= DECEL;  	// 减速状态.
		srd[Plat][Axis].step_delay 	= 1000;		// 短延时	
	}
	else if(step == 0)
	{
		return;
	}
	else if(step != 0)  // 如果目标运动步数不为0
	{
		// 在电机控制专题指导手册有详细的计算及推导过程

		/** 使用do循环来确定合适的定时器预分频,确保启动的时候速度足够低,但
		 *  预分频越低,则可以达到的最高速度则越低.
		 */
		do
		{ 	// 这一步是调整定时器预分频,使计算结果不会溢出
			__HAL_TIM_SET_PRESCALER(htimx[Plat], Prescaler++);
			SM_LSC_TIMx_Freq 		= ((float)SystemCoreClock / Prescaler);      
			SM_LSC_TIMx_Freq_148 	= (((SM_LSC_TIMx_Freq * 0.676f) / 10.0f));
		  
			// 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2

			srd[Plat][Axis].step_delay = round((SM_LSC_TIMx_Freq_148 * sqrt(A_SQ / accel)) / 10);//C0,初始速度的定时器值
				
		} while(srd[Plat][Axis].step_delay > (uint32_t)0xFFFF);
		
		A_T_x10 = ((float)(10*ALPHA*SM_LSC_TIMx_Freq));
		// 设置最大速度极限, 计算得到min_delay用于定时器的计数器的值。

		srd[Plat][Axis].min_delay = (int32_t)(A_T_x10 / speed);

		// 通过计算第一个(c0) 的步进延时来设定加速度，其中accel单位为0.1rad/sec^2

		srd[Plat][Axis].step_delay = (int32_t)((T1_FREQ_148 * sqrt(A_SQ / accel)) / 10);//C0,初始速度的定时器值

		/*计算加减速需要的参数*/
		// 计算多少步之后达到最大速度的限制
		max_s_lim = (uint32_t)(speed * speed / (A_x200 * accel / 10));
		// 如果达到最大速度小于0.5步，我们将四舍五入为0
		// 但实际我们必须移动至少一步才能达到想要的速度
		if(max_s_lim == 0)
		{
			max_s_lim = 1;
		}
			
		// 计算多少步之后我们必须开始减速

		accel_lim = (uint32_t)(step * decel / (accel + decel));
		// 我们必须加速至少1步才能才能开始减速.
		if(accel_lim == 0)
		{
			accel_lim = 1;
		}
		// 使用限制条件我们可以计算出减速阶段步数
		if(accel_lim <= max_s_lim)
		{
			srd[Plat][Axis].decel_val = accel_lim - step;
		}
		else
		{
			srd[Plat][Axis].decel_val = -(max_s_lim * accel / decel);
		}
		// 当只剩下一步我们必须减速
		if(srd[Plat][Axis].decel_val == 0)
		{
			srd[Plat][Axis].decel_val = -1;
		}

		// 计算开始减速时的步数
		srd[Plat][Axis].decel_start = step + srd[Plat][Axis].decel_val;

		// 如果最大速度很慢，我们就不需要进行加速运动
		if(srd[Plat][Axis].step_delay <= srd[Plat][Axis].min_delay)
		{
			srd[Plat][Axis].step_delay 	= srd[Plat][Axis].min_delay;
			srd[Plat][Axis].run_state 	= RUN;
		}
		else
		{
			srd[Plat][Axis].run_state = ACCEL;
		}    
		// 复位加速度计数值
		srd[Plat][Axis].accel_count = 0;
		
	}
	MotionStatus[Plat][Axis] = 1;
  
	tim_count = __HAL_TIM_GET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel);
	tim_count += (srd[Plat][Axis].step_delay >> 1) ;
	__HAL_TIM_SET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel, (uint16_t)tim_count); // 设置定时器比较值
	HAL_TIM_OC_Start_IT(htimx[Plat], Timer[Plat][Axis].Channel);
	STEPMOTOR_OUTPUT_ENABLE(Plat, Axis);
}


/** 函数功能: 定点移动
  * 输入参数: targert_step:目标的位置
  *			      accel:加速度
  *			      decel:减速度
  *			      speed:最大速度
  * 返 回 值: 无
  * 说    明: 实现定点移动,输入目标位置相对于原点的步数,
  *  		      以加速度加速到最大速度后匀速寻找目标位置,
  *			      然后在合适的位置减速,到达目标位置.
  */
void STEPMOTOR_AxisMoveAbs(uint8_t Plat, uint8_t Axis, int32_t targert_step, uint32_t accel, uint32_t decel, uint32_t speed)
{
	int32_t rel_step = 0;
	int8_t dir = -1;                        //回原点的方向控制
	
	rel_step = step_position[Plat][Axis] - targert_step ; 	//获取当前位置和目标位置之间的步数值
	
	if(rel_step == 0)	
	{
		dir = 0;
	}
	STEPMOTOR_AxisMoveRel(Plat, Axis, dir * rel_step, accel, decel, speed);
	
	return;
}


/** 函数功能: 相对移动一定距离;单位:mm
  * 输入参数: distance  需要移动的距离,以正负区分方向,正数就是正方向,
  *                    负数就是反方向
  *            accel  加速度
  *            decel  减速度
  *            speed  最大速度
  * 返 回 值: 无
  * 说    明: 从当前位置上在导轨上移动一定的距离,以参数distance的正负区分方向
 */
void STEPMOTOR_DisMoveRel(__IO uint8_t Plat, __IO uint8_t Axis, __IO int16_t distance, __IO uint32_t accel, __IO uint32_t decel, __IO uint32_t speed)
{
	__IO int32_t step;
	step = distance * UNIT_STEP_MM;//获得步进目标距离的步数
	
	STEPMOTOR_AxisMoveRel(Plat, Axis, step, accel, decel, speed);
	
	return;
}

/**
  * 函数功能: 移动到目标位置;单位:mm
  * 输入参数: Target_Dis  目标坐标位置
  *           accel  加速度
  *           decel  减速度
  *           speed  最大速度
  * 返 回 值: 最大速度
  * 说    明: 移动到给定的坐标位置,根据当前的位置计算出与目标位置的距离,
  *           并移动到目标位置,
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
	// 总移动步数计数器
	__IO static uint32_t step_count[4]			= {0, 0, 0, 0};
	// 记录new_step_delay中的余数，提高下一步计算的精度
	__IO static int32_t rest[4]					= {0, 0, 0, 0};
	//定时器使用翻转模式，需要进入两次中断才输出一个完整脉冲

	// 设置比较值
	tim_count = __HAL_TIM_GET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel);
	tim_count += (srd[Plat][Axis].step_delay >> 1);
	__HAL_TIM_SET_COMPARE(htimx[Plat], Timer[Plat][Axis].Channel, (uint16_t)tim_count);
	
	i[Axis]++;       // 定时器中断次数计数值
	if(i[Axis] == 2)
	{
		i[Axis] = 0;
		
		/* 判断是否处于边缘运动 */
		if(Axis != AXIS_R)
		{
			/* 判断是否处于边缘运动 */
			if(srd[Plat][Axis].dir == MOTOR_DIR_CCW)
			{
				if(LimNega[Plat][Axis] == TRUE)		//逆时针且 遇到反转的极限位置,停止运动
				{
					srd[Plat][Axis].run_state 	= STOP;
					MotionStatus[Plat][Axis] 	= STOP;
					LimPosi[Plat][Axis] 		= FALSE;//正转的极限标志位应为FALSE
				}
				else 
				{
					LimPosi[Plat][Axis] = FALSE;	  	//在逆时针方向,在碰到反转的极限位置之前,正转的极限标志位应为FALSE
				}
			}
			else 
			{
				if(LimPosi[Plat][Axis] == TRUE)		
				{
					srd[Plat][Axis].run_state 	= STOP;
					MotionStatus[Plat][Axis] 	= STOP;
					LimNega[Plat][Axis] 		= FALSE;//在顺时针方向,碰到极限位置,反转极限标志位应为False
				}
				else 
				{
					LimNega[Plat][Axis] = FALSE;  	//在顺时针方向,在碰到正转的极限位置之前,反转的极限标志位应为FALSE
				}
			}
		}
		switch(srd[Plat][Axis].run_state) 		// 加减速曲线阶段
		{
			case STOP:
			{
				// 关闭通道
				TIM_CCxChannelCmd(STEPMOTOR_TIMx[Plat], Timer[Plat][Axis].Channel, TIM_CCx_DISABLE);       
				__HAL_TIM_CLEAR_FLAG(htimx[Plat], Timer[Plat][Axis].Flag_CCx);
				HAL_TIM_OC_Stop_IT(htimx[Plat], Timer[Plat][Axis].Channel);
			  
				MotionStatus[Plat][Axis] 		= STOP;  	//  电机为停止状态
				step_count[Axis] 			= 0;  		// 清零步数计数器
				rest[Axis]					= 0;        // 清零余值
				last_accel_delay[Axis]		= 0;
				srd[Plat][Axis].accel_count 	= 0;
				srd[Plat][Axis].step_delay 	= 0;
				srd[Plat][Axis].min_delay 		= 0;
			
				if( PreAdjusting_Work == 1)
				{
					PreAdjust_Done[Plat][Axis] = 1;
				}
				
				
				
				//轴承平台调整完位置时 检测的P3R顶杆电机
				if((Plat == P3)&&(Axis == AXIS_R))
				{
					if(BearingPlatform_AdjustDone == 1)
					{
						BearingPlatform_AdjustDone = 2;						
					}
					break;
				}
				
				//相机平台电机调整到位检测
				if(CameraMotor_AdjustType!=0)
				{
					CameraMotor_AdjustDone[Plat][Axis] = 1;
				}
							
				break;
			}
			case ACCEL:
			{
				step_count[Axis]++;      			// 步数加1
				if(srd[Plat][Axis].dir == MOTOR_DIR_CW)
				{		  	
					step_position[Plat][Axis]++; 	  		// 绝对位置加1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]++;
				}
				else
				{
					step_position[Plat][Axis]--; 	  		// 绝对位置减1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]--;
				}
				srd[Plat][Axis].accel_count++; 	  		// 加速计数值加1
				
				//计算新(下)一步脉冲周期(时间间隔)
				new_step_delay 	= srd[Plat][Axis].step_delay - (((2 * srd[Plat][Axis].step_delay) + rest[Axis]) / (4 * srd[Plat][Axis].accel_count + 1));	
				// 计算余数，下次计算补上余数，减少误差
				rest[Axis] = ((2 * srd[Plat][Axis].step_delay) + rest[Axis]) % (4 * srd[Plat][Axis].accel_count + 1);							
				
				if(step_count[Axis] >= srd[Plat][Axis].decel_start)			// 检查是否应该开始减速
				{
					srd[Plat][Axis].accel_count 	= srd[Plat][Axis].decel_val; 		// 加速计数值为减速阶段计数值的初始值
					srd[Plat][Axis].run_state 		= DECEL;           	      	// 下个脉冲进入减速阶段
				}
				else if(new_step_delay <= srd[Plat][Axis].min_delay)  		// 检查是否到达期望的最大速度
				{
					srd[Plat][Axis].accel_count 	= srd[Plat][Axis].decel_val; 		// 加速计数值为减速阶段计数值的初始值
					last_accel_delay[Axis] 		= new_step_delay; 			// 保存加速过程中最后一次延时（脉冲周期）
					new_step_delay 				= srd[Plat][Axis].min_delay;    	// 使用min_delay（对应最大速度speed）
					rest[Axis]					= 0;                          	// 清零余值
					srd[Plat][Axis].run_state 		= RUN;               		// 设置为匀速运行状态
				}	
				last_accel_delay[Axis] = new_step_delay; 	  			// 保存加速过程中最后一次延时（脉冲周期）
				break;
			}
			case RUN:
			{
				step_count[Axis]++; 		 		// 步数加1
				if(srd[Plat][Axis].dir == MOTOR_DIR_CW)
				{	  	
					step_position[Plat][Axis]++; 			// 绝对位置加1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]++;
				}
				else
				{
					step_position[Plat][Axis]--; 			// 绝对位置减1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]--;
				}
				
				new_step_delay = srd[Plat][Axis].min_delay;       			// 使用min_delay（对应最大速度speed）
					
				if(step_count[Axis] >= srd[Plat][Axis].decel_start)   		// 需要开始减速
				{
					srd[Plat][Axis].accel_count 	= srd[Plat][Axis].decel_val;  	// 减速步数做为加速计数值
					new_step_delay				= last_accel_delay[Axis];	// 加阶段最后的延时做为减速阶段的起始延时(脉冲周期)
					srd[Plat][Axis].run_state 		= DECEL;            		// 状态改变为减速
				}
				break;
			}
			case DECEL:
			{
				step_count[Axis]++; 		 		// 步数加1
				if(srd[Plat][Axis].dir == MOTOR_DIR_CW)
				{		  	
					step_position[Plat][Axis]++; 			// 绝对位置加1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]++;
				}
				else
				{
					step_position[Plat][Axis]--; 			// 绝对位置减1
					if( 0 == (step_position[Plat][Axis] % UNIT_STEP_MM )) 
						location[Plat][Axis]--;
				}
				srd[Plat][Axis].accel_count++;
				new_step_delay = srd[Plat][Axis].step_delay - (((2 * srd[Plat][Axis].step_delay) + rest[Axis]) / (4 * srd[Plat][Axis].accel_count + 1)); 	//计算新(下)一步脉冲周期(时间间隔)
				rest[Axis] = ((2 * srd[Plat][Axis].step_delay) + rest[Axis]) % (4 * srd[Plat][Axis].accel_count + 1);								// 计算余数，下次计算补上余数，减少误差
				
				//检查是否为最后一步
				if(srd[Plat][Axis].accel_count >= 0)
				{
					StopMotor(Plat, Axis);
				}
				break;
			}
			case DECEL_MEDLE:
			{
				new_step_delay = srd[Plat][Axis].step_delay - (((2 * srd[Plat][Axis].step_delay) + rest[Axis])/(4 * srd[Plat][Axis].accel_count + 1)); //计算新(下)一步脉冲周期(时间间隔)
				rest[Axis] = ((2 * srd[Plat][Axis].step_delay) + rest[Axis]) % (4 * srd[Plat][Axis].accel_count + 1);// 计算余数，下次计算补上余数，减少误差
				//检查是否为最后一步
				if(new_step_delay >= srd[Plat][Axis].medle_delay )
				{
					srd[Plat][Axis].min_delay 	= srd[Plat][Axis].medle_delay;
					step_count[Axis] 						= 0;  		// 清零步数计数器
					rest[Axis] 							= 0;        // 清零余值
					srd[Plat][Axis].run_state 	= RUN;              
				}
				break;
			}
			default:
				break;
		}
		if( (new_step_delay >> 1) > 0xFFFF)
			new_step_delay 			= 0x1FFFF;
			srd[Plat][Axis].step_delay = new_step_delay; // 为下个(新的)延时(脉冲周期)赋值
	}
	
	return;
}


/**
  * 函数功能: 停止电机转动
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 停止定时器中断和输出，并且清除加减速的参数。
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

//回城部分

void ResponsePoint(uint8_t Plat , uint8_t Axis)
{
		StopMotor(Plat, Axis);
		step_position[Plat][Axis]=0;
		location[Plat][Axis]=0;
		memset((void*)&srd[Plat][Axis], 0, sizeof(srd[Plat][Axis]));		
    return;	
}


