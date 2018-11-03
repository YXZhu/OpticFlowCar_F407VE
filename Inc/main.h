/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "SEGGER_RTT.h"

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define KEY1_Pin GPIO_PIN_3
#define KEY1_GPIO_Port GPIOE
#define KEY0_Pin GPIO_PIN_4
#define KEY0_GPIO_Port GPIOE
#define D3_Pin GPIO_PIN_7
#define D3_GPIO_Port GPIOA
#define SIO_C_Pin GPIO_PIN_4
#define SIO_C_GPIO_Port GPIOC
#define SIO_D_Pin GPIO_PIN_5
#define SIO_D_GPIO_Port GPIOC
#define F_CS_Pin GPIO_PIN_0
#define F_CS_GPIO_Port GPIOB
#define CameraLight_Pin GPIO_PIN_9
#define CameraLight_GPIO_Port GPIOE
#define GPIO1_3_Pin GPIO_PIN_10
#define GPIO1_3_GPIO_Port GPIOE
#define GPIO1_3_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT3_Pin GPIO_PIN_11
#define XSHUT3_GPIO_Port GPIOE
#define GPIO1_2_Pin GPIO_PIN_12
#define GPIO1_2_GPIO_Port GPIOE
#define GPIO1_2_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT2_Pin GPIO_PIN_13
#define XSHUT2_GPIO_Port GPIOE
#define GPIO1_1_Pin GPIO_PIN_14
#define GPIO1_1_GPIO_Port GPIOE
#define GPIO1_1_EXTI_IRQn EXTI15_10_IRQn
#define XSHUT1_Pin GPIO_PIN_15
#define XSHUT1_GPIO_Port GPIOE
#define NSS_Pin GPIO_PIN_12
#define NSS_GPIO_Port GPIOB
#define SPI2_MISO_Pin GPIO_PIN_14
#define SPI2_MISO_GPIO_Port GPIOB
#define SPI2_MOSI_Pin GPIO_PIN_15
#define SPI2_MOSI_GPIO_Port GPIOB
#define MOTION_Pin GPIO_PIN_8
#define MOTION_GPIO_Port GPIOD
#define MOTION_EXTI_IRQn EXTI9_5_IRQn
#define RESET_Pin GPIO_PIN_9
#define RESET_GPIO_Port GPIOD
#define PWDN_Pin GPIO_PIN_14
#define PWDN_GPIO_Port GPIOD
#define RESTE_Pin GPIO_PIN_15
#define RESTE_GPIO_Port GPIOD
#define CS_Pin GPIO_PIN_5
#define CS_GPIO_Port GPIOD
#define RST_Pin GPIO_PIN_6
#define RST_GPIO_Port GPIOD
#define A0_RS_Pin GPIO_PIN_7
#define A0_RS_GPIO_Port GPIOD
#define ADO_Pin GPIO_PIN_0
#define ADO_GPIO_Port GPIOE
#define INT_Pin GPIO_PIN_1
#define INT_GPIO_Port GPIOE
#define INT_EXTI_IRQn EXTI1_IRQn

/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */
#define CCMRAM  __attribute__((section("ccmram")))

union n16bit
{
  unsigned char c[2];    // c[0] = LO byte, c[1] = HI byte
  unsigned short i;
  signed short si;
};

//#define CCMRAML  __attribute__((at("ccmram"))) 
/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
