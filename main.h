/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED2_Pin GPIO_PIN_5
#define LED2_GPIO_Port GPIOE
#define LED3_Pin GPIO_PIN_6
#define LED3_GPIO_Port GPIOE
#define P4_R_ORGLIM_Pin GPIO_PIN_8
#define P4_R_ORGLIM_GPIO_Port GPIOI
#define P2_Y_NEGLIM_Pin GPIO_PIN_0
#define P2_Y_NEGLIM_GPIO_Port GPIOF
#define P1_M3_DIR_Pin GPIO_PIN_1
#define P1_M3_DIR_GPIO_Port GPIOF
#define P1_M3_EN_Pin GPIO_PIN_2
#define P1_M3_EN_GPIO_Port GPIOF
#define P4_M3_EN_Pin GPIO_PIN_3
#define P4_M3_EN_GPIO_Port GPIOF
#define P4_M4_EN_Pin GPIO_PIN_4
#define P4_M4_EN_GPIO_Port GPIOF
#define P3_X_POSLIM_Pin GPIO_PIN_5
#define P3_X_POSLIM_GPIO_Port GPIOF
#define P4_M4_DIR_Pin GPIO_PIN_6
#define P4_M4_DIR_GPIO_Port GPIOF
#define P4_M3_DIR_Pin GPIO_PIN_7
#define P4_M3_DIR_GPIO_Port GPIOF
#define P3_X_NEGLIM_Pin GPIO_PIN_8
#define P3_X_NEGLIM_GPIO_Port GPIOF
#define P3_Y_POSLIM_Pin GPIO_PIN_9
#define P3_Y_POSLIM_GPIO_Port GPIOF
#define P3_Y_NEGLIM_Pin GPIO_PIN_10
#define P3_Y_NEGLIM_GPIO_Port GPIOF
#define P3_M3_EN_Pin GPIO_PIN_0
#define P3_M3_EN_GPIO_Port GPIOC
#define P3_M4_DIR_Pin GPIO_PIN_3
#define P3_M4_DIR_GPIO_Port GPIOC
#define P2_M2_DIR_Pin GPIO_PIN_0
#define P2_M2_DIR_GPIO_Port GPIOA
#define P4_Y_NEGLIM_Pin GPIO_PIN_2
#define P4_Y_NEGLIM_GPIO_Port GPIOH
#define P4_X_NEGLIM_Pin GPIO_PIN_3
#define P4_X_NEGLIM_GPIO_Port GPIOH
#define P4_Y_POSLIM_Pin GPIO_PIN_4
#define P4_Y_POSLIM_GPIO_Port GPIOH
#define P4_X_POSLIM_Pin GPIO_PIN_5
#define P4_X_POSLIM_GPIO_Port GPIOH
#define P2_M3_DIR_Pin GPIO_PIN_3
#define P2_M3_DIR_GPIO_Port GPIOA
#define P2_M2_EN_Pin GPIO_PIN_4
#define P2_M2_EN_GPIO_Port GPIOA
#define P2_M1_PUL_Pin GPIO_PIN_6
#define P2_M1_PUL_GPIO_Port GPIOA
#define P2_M3_PUL_Pin GPIO_PIN_0
#define P2_M3_PUL_GPIO_Port GPIOB
#define P2_M4_PUL_Pin GPIO_PIN_1
#define P2_M4_PUL_GPIO_Port GPIOB
#define P1_M2_EN_Pin GPIO_PIN_11
#define P1_M2_EN_GPIO_Port GPIOF
#define OUTPUT1_Pin GPIO_PIN_12
#define OUTPUT1_GPIO_Port GPIOF
#define OUTPUT2_Pin GPIO_PIN_13
#define OUTPUT2_GPIO_Port GPIOF
#define OUTPUT3_Pin GPIO_PIN_14
#define OUTPUT3_GPIO_Port GPIOF
#define OUTPUT4_Pin GPIO_PIN_15
#define OUTPUT4_GPIO_Port GPIOF
#define P1_X_POSLIM_Pin GPIO_PIN_0
#define P1_X_POSLIM_GPIO_Port GPIOG
#define P1_X_POSLIM_EXTI_IRQn EXTI0_IRQn
#define P1_X_NEGLIM_Pin GPIO_PIN_1
#define P1_X_NEGLIM_GPIO_Port GPIOG
#define P1_X_NEGLIM_EXTI_IRQn EXTI1_IRQn
#define P3_M1_EN_Pin GPIO_PIN_7
#define P3_M1_EN_GPIO_Port GPIOE
#define P3_M1_DIR_Pin GPIO_PIN_8
#define P3_M1_DIR_GPIO_Port GPIOE
#define P4_Z_ORGLIM_Pin GPIO_PIN_9
#define P4_Z_ORGLIM_GPIO_Port GPIOE
#define P4_Z_POSLIM_Pin GPIO_PIN_10
#define P4_Z_POSLIM_GPIO_Port GPIOE
#define P4_M2_EN_Pin GPIO_PIN_11
#define P4_M2_EN_GPIO_Port GPIOE
#define P4_M2_DIR_Pin GPIO_PIN_12
#define P4_M2_DIR_GPIO_Port GPIOE
#define P4_M1_EN_Pin GPIO_PIN_13
#define P4_M1_EN_GPIO_Port GPIOE
#define P4_M4_PUL_Pin GPIO_PIN_14
#define P4_M4_PUL_GPIO_Port GPIOE
#define P2_R_ORGLIM_Pin GPIO_PIN_15
#define P2_R_ORGLIM_GPIO_Port GPIOE
#define P3_R_ORGLIM_Pin GPIO_PIN_6
#define P3_R_ORGLIM_GPIO_Port GPIOH
#define P1_M4_EN_Pin GPIO_PIN_7
#define P1_M4_EN_GPIO_Port GPIOH
#define LED1_Pin GPIO_PIN_9
#define LED1_GPIO_Port GPIOH
#define P2_M4_DIR_Pin GPIO_PIN_10
#define P2_M4_DIR_GPIO_Port GPIOH
#define P2_M1_DIR_Pin GPIO_PIN_11
#define P2_M1_DIR_GPIO_Port GPIOH
#define P2_M1_EN_Pin GPIO_PIN_12
#define P2_M1_EN_GPIO_Port GPIOH
#define P3_Z_NEGLIM_Pin GPIO_PIN_13
#define P3_Z_NEGLIM_GPIO_Port GPIOB
#define P3_Z_NEGLIM_EXTI_IRQn EXTI15_10_IRQn
#define P3_Z_ORGLIM_Pin GPIO_PIN_14
#define P3_Z_ORGLIM_GPIO_Port GPIOB
#define P3_Z_POSLIM_Pin GPIO_PIN_15
#define P3_Z_POSLIM_GPIO_Port GPIOB
#define P2_Z_NEGLIM_Pin GPIO_PIN_8
#define P2_Z_NEGLIM_GPIO_Port GPIOD
#define P2_Z_POSLIM_Pin GPIO_PIN_9
#define P2_Z_POSLIM_GPIO_Port GPIOD
#define P2_Z_ORGLIM_Pin GPIO_PIN_10
#define P2_Z_ORGLIM_GPIO_Port GPIOD
#define P1_M2_DIR_Pin GPIO_PIN_11
#define P1_M2_DIR_GPIO_Port GPIOD
#define P3_M1_PUL_Pin GPIO_PIN_12
#define P3_M1_PUL_GPIO_Port GPIOD
#define P3_M2_PUL_Pin GPIO_PIN_13
#define P3_M2_PUL_GPIO_Port GPIOD
#define P4_Z_NEGLIM_Pin GPIO_PIN_14
#define P4_Z_NEGLIM_GPIO_Port GPIOD
#define P4_Z_NEGLIM_EXTI_IRQn EXTI15_10_IRQn
#define P3_M4_PUL_Pin GPIO_PIN_15
#define P3_M4_PUL_GPIO_Port GPIOD
#define P1_Y_POSLIM_Pin GPIO_PIN_2
#define P1_Y_POSLIM_GPIO_Port GPIOG
#define P1_Y_POSLIM_EXTI_IRQn EXTI2_IRQn
#define P1_Y_NEGLIM_Pin GPIO_PIN_3
#define P1_Y_NEGLIM_GPIO_Port GPIOG
#define P1_Y_NEGLIM_EXTI_IRQn EXTI3_IRQn
#define P1_Z_ORGLIM_Pin GPIO_PIN_4
#define P1_Z_ORGLIM_GPIO_Port GPIOG
#define P1_Z_ORGLIM_EXTI_IRQn EXTI4_IRQn
#define P1_Z_POSLIM_Pin GPIO_PIN_5
#define P1_Z_POSLIM_GPIO_Port GPIOG
#define P1_Z_POSLIM_EXTI_IRQn EXTI9_5_IRQn
#define P1_Z_NEGLIM_Pin GPIO_PIN_6
#define P1_Z_NEGLIM_GPIO_Port GPIOG
#define P1_Z_NEGLIM_EXTI_IRQn EXTI9_5_IRQn
#define P1_R_ORGLIM_Pin GPIO_PIN_7
#define P1_R_ORGLIM_GPIO_Port GPIOG
#define P1_R_ORGLIM_EXTI_IRQn EXTI9_5_IRQn
#define TRIG1_Pin GPIO_PIN_8   //G8 滚轴里的传感器
#define TRIG1_GPIO_Port GPIOG
#define TRIG1_EXTI_IRQn EXTI9_5_IRQn
#define P2_M2_PUL_Pin GPIO_PIN_7
#define P2_M2_PUL_GPIO_Port GPIOC
#define P1_M4_DIR_Pin GPIO_PIN_8
#define P1_M4_DIR_GPIO_Port GPIOC
#define P2_X_POSLIM_Pin GPIO_PIN_9
#define P2_X_POSLIM_GPIO_Port GPIOC
#define P4_M1_PUL_Pin GPIO_PIN_8
#define P4_M1_PUL_GPIO_Port GPIOA
#define P4_M2_PUL_Pin GPIO_PIN_9
#define P4_M2_PUL_GPIO_Port GPIOA
#define P4_M3_PUL_Pin GPIO_PIN_10
#define P4_M3_PUL_GPIO_Port GPIOA
#define P3_M4_EN_Pin GPIO_PIN_13
#define P3_M4_EN_GPIO_Port GPIOH
#define P3_M3_DIR_Pin GPIO_PIN_14
#define P3_M3_DIR_GPIO_Port GPIOH
#define P1_M4_PUL_Pin GPIO_PIN_2
#define P1_M4_PUL_GPIO_Port GPIOI
#define P2_M3_EN_Pin GPIO_PIN_15
#define P2_M3_EN_GPIO_Port GPIOA
#define P3_M2_EN_Pin GPIO_PIN_0
#define P3_M2_EN_GPIO_Port GPIOD
#define P3_M2_DIR_Pin GPIO_PIN_1
#define P3_M2_DIR_GPIO_Port GPIOD
#define P1_M1_DIR_Pin GPIO_PIN_3
#define P1_M1_DIR_GPIO_Port GPIOD
#define P4_M1_DIR_Pin GPIO_PIN_4
#define P4_M1_DIR_GPIO_Port GPIOD
#define P2_Y_POSLIM_Pin GPIO_PIN_5
#define P2_Y_POSLIM_GPIO_Port GPIOD
#define P1_M1_EN_Pin GPIO_PIN_7
#define P1_M1_EN_GPIO_Port GPIOD
#define TRIG2_Pin GPIO_PIN_9             //档杆
#define TRIG2_GPIO_Port GPIOG
#define TRIG2_EXTI_IRQn EXTI9_5_IRQn
#define TRIG3_Pin GPIO_PIN_10
#define TRIG3_GPIO_Port GPIOG
#define TRIG3_EXTI_IRQn EXTI15_10_IRQn
#define P2_X_NEGLIM_Pin GPIO_PIN_12
#define P2_X_NEGLIM_GPIO_Port GPIOG
#define P2_X_NEGLIM_EXTI_IRQn EXTI15_10_IRQn
#define TRIG4_Pin GPIO_PIN_15
#define TRIG4_GPIO_Port GPIOG
#define TRIG4_EXTI_IRQn EXTI15_10_IRQn
#define P2_M4_EN_Pin GPIO_PIN_3
#define P2_M4_EN_GPIO_Port GPIOB
#define P3_M3_PUL_Pin GPIO_PIN_8
#define P3_M3_PUL_GPIO_Port GPIOB
#define P1_M1_PUL_Pin GPIO_PIN_5
#define P1_M1_PUL_GPIO_Port GPIOI
#define P1_M2_PUL_Pin GPIO_PIN_6
#define P1_M2_PUL_GPIO_Port GPIOI
#define P1_M3_PUL_Pin GPIO_PIN_7
#define P1_M3_PUL_GPIO_Port GPIOI
#define BEEP_GPIO_Pin	GPIO_PIN_10
#define BEEP_GPIO_Port	GPIOI




#define  ROLLERON      1
#define  ROLLEROFF    0

//串口发送给上位机还是下位机从机谢列
#define COMPUTER 1
#define SLAVE  			 0

//电机类别的判断
#define CAMERA_MOTOR 									0
#define BEARINGPLATFORM_MOTOR 1

//轴承平台电机回城判断
#define SPLINTWIDTH_MOTOR 	0  
#define SPLINTHEIGHT_MOTOR 	1  
#define GUARD_MOTOR 							2


//档杆电机控制
#define GUARDSTANDBY 0
#define GUARDUPING 1
#define GUARDDOWNING 2
#define GUARDBUSY_UPING  3
#define GUARDBUSY_DOWNING 4

//轴承平台电机回原点HomeType
#define BLOCKEXPEND_1  1
#define BLOCKEXPEND_2  2
#define BLOCKHEIGHT_1   3
#define BLOCKHEIGHT_2   4
#define GUARDHOME_1     5
#define GUARDHOME_2     6

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
