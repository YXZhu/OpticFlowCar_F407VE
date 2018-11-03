/**
  ******************************************************************************
  * File Name          : I2C.c
  * Description        : This file provides code for the configuration
  *                      of the I2C instances.
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

/* Includes ------------------------------------------------------------------*/
#include "i2c.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */
#include "dwt_stm32_delay.h"
/* USER CODE END 0 */

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* I2C2 init function */
void MX_I2C2_Init(void)
{

  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_16_9;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_I2C_MspInit(I2C_HandleTypeDef* i2cHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspInit 0 */
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9) || !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
	  {

		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
		  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
		  {	
			  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			  DWT_Delay_us(10);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			  DWT_Delay_us(10);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		  DWT_Delay_us(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		  DWT_Delay_us(10);
		  //HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
		  //HAL_GPIO_DeInit(GPIOB,GPIO_PIN_8);
		  //i2cHandle->Instance->CR1 = I2C_CR1_SWRST;
		  //i2cHandle->Instance->CR1= 0; 
		  
	  }
  /* USER CODE END I2C1_MspInit 0 */
  
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C1 clock enable */
    __HAL_RCC_I2C1_CLK_ENABLE();

    /* I2C1 interrupt Init */
    HAL_NVIC_SetPriority(I2C1_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_SetPriority(I2C1_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspInit 1 */
	
  /* USER CODE END I2C1_MspInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspInit 0 */
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)|| !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11))
	  {		  
		  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);
		  while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11))
		  {	
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			  DWT_Delay_us(10);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			  DWT_Delay_us(10);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		  DWT_Delay_us(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		  DWT_Delay_us(10);

	  }
  /* USER CODE END I2C2_MspInit 0 */
  
    /**I2C2 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* I2C2 clock enable */
    __HAL_RCC_I2C2_CLK_ENABLE();

    /* I2C2 interrupt Init */
    HAL_NVIC_SetPriority(I2C2_EV_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_SetPriority(I2C2_ER_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspInit 1 */

  /* USER CODE END I2C2_MspInit 1 */
  }
}

void HAL_I2C_MspDeInit(I2C_HandleTypeDef* i2cHandle)
{

  if(i2cHandle->Instance==I2C1)
  {
  /* USER CODE BEGIN I2C1_MspDeInit 0 */

  /* USER CODE END I2C1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C1_CLK_DISABLE();
  
    /**I2C1 GPIO Configuration    
    PB8     ------> I2C1_SCL
    PB9     ------> I2C1_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);

    /* I2C1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C1_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C1_ER_IRQn);
  /* USER CODE BEGIN I2C1_MspDeInit 1 */

  /* USER CODE END I2C1_MspDeInit 1 */
  }
  else if(i2cHandle->Instance==I2C2)
  {
  /* USER CODE BEGIN I2C2_MspDeInit 0 */

  /* USER CODE END I2C2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_I2C2_CLK_DISABLE();
  
    /**I2C2 GPIO Configuration    
    PB10     ------> I2C2_SCL
    PB11     ------> I2C2_SDA 
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* I2C2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_DisableIRQ(I2C2_ER_IRQn);
  /* USER CODE BEGIN I2C2_MspDeInit 1 */

  /* USER CODE END I2C2_MspDeInit 1 */
  }
} 

/* USER CODE BEGIN 1 */
#include <string.h>
typedef struct {
  uint8_t Buf[32];
	uint8_t addr;
	uint8_t reg;
	uint8_t len;
} MEM_BLOCK;

osSemaphoreId I2c1RxComplete;
osSemaphoreId I2c1TxComplete;
osPoolId I2CdataSave;
osMutexId I2C1Mutex; //创建i2c互斥信号量 独占资源；
osMutexId I2C2Mutex;
void I2CFreertosInit(void)
{
	osSemaphoreDef(I2c1RxComplete);
	I2c1RxComplete = osSemaphoreCreate(osSemaphore(I2c1RxComplete),1);
	
	osSemaphoreDef(I2c1TxComplete);
	I2c1TxComplete = osSemaphoreCreate(osSemaphore(I2c1TxComplete),1);
	
	osMutexDef(I2C1Mutex);
	I2C1Mutex = osMutexCreate(osMutex(I2C1Mutex));
	
	osMutexDef(I2C2Mutex);
	I2C2Mutex = osMutexCreate(osMutex(I2C2Mutex));	
	
	osPoolDef(I2CdataSave,8,MEM_BLOCK);
	I2CdataSave = osPoolCreate(osPool(I2CdataSave));
	
}

void I2CError(I2C_HandleTypeDef *hi2c)
{
	//HAL_I2C_DeInit(hi2c);
	//HAL_I2C_Init(hi2c);
	HAL_I2C_ErrorCallback(hi2c);
}
MEM_BLOCK i2cmem;
int i2c1Write(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data)
{

	addr <<=1;  /* 直接对7bit地址左移 必须！！*/
	//if(HAL_I2C_Master_Transmit(&hi2c1,addr,t,len,0xff)!= HAL_OK)
	//osMutexWait(I2C1Mutex,100);
	
//	i2cmem.addr = addr;
//	i2cmem.reg = reg;
//	i2cmem.len = len;
//	memcpy(i2cmem.Buf,data,i2cmem.len);
//	//mem = osPoolAlloc(I2CdataSave);
//	/* 大于2个字节用DMA，DMA可以说是特效药，“屡试不爽”。不过要注意，接收大于或等于2个字节时才能使用DMA，不然不能产生EOT-1事件导致NACK不能正确发送。*/
//	if(HAL_I2C_Mem_Write_DMA(&hi2c1,i2cmem.addr,i2cmem.reg,I2C_MEMADD_SIZE_8BIT,i2cmem.Buf,i2cmem.len) != HAL_OK)
//	{
//		//SEGGER_RTT_printf(0, "I2C Write ERROR\n");
//		HAL_I2C_Master_Abort_IT(&hi2c1,i2cmem.addr);
//		I2CError(&hi2c1);
//		osMutexRelease(I2C1Mutex);
//		return -1;
//	}
//	if(osSemaphoreWait(I2c1TxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);	
	//osPoolFree(I2CdataSave,mem);
	if(HAL_I2C_Mem_Write(&hi2c1,(uint16_t)addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)data,len,10) != HAL_OK)
	{
		//SEGGER_RTT_printf(0, "I2C Write ERROR\n");
		I2CError(&hi2c1);
		//osMutexRelease(I2C1Mutex);
		return -1;
	}	
	//HAL_Delay(1);
	
		//HAL_I2C_Master_Abort_IT(&hi2c1,addr);
	//osDelay(1);
	//osMutexRelease(I2C1Mutex);
	return 0;
	
}

int i2c1Read(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{

	addr <<=1; /* 直接对7bit地址左移 必须！！*/
	//if(HAL_I2C_Master_Receive(&hi2c1,addr,t,len,0xff)!= HAL_OK)
	//osMutexWait(I2C1Mutex,100);
//	i2cmem.addr = addr;
//	i2cmem.reg = reg;
//	i2cmem.len = len;
//	//memset(bufs,0,len1);
//	if(HAL_I2C_Mem_Read_DMA(&hi2c1,(uint16_t)i2cmem.addr,i2cmem.reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)i2cmem.Buf,i2cmem.len)!= HAL_OK)
//	{
//		//SEGGER_RTT_printf(0, "I2C Read ERROR\n");
//		HAL_I2C_Master_Abort_IT(&hi2c1,i2cmem.addr);
//		I2CError(&hi2c1);
//		osMutexRelease(I2C1Mutex);
//		return -1;
//	}
//	
//	//HAL_Delay(2);
//	if(osSemaphoreWait(I2c1RxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);		
//	memcpy(buf,i2cmem.Buf,i2cmem.len);

	if(HAL_I2C_Mem_Read(&hi2c1,(uint16_t)addr,reg,I2C_MEMADD_SIZE_8BIT,(uint8_t*)buf,len,10)!= HAL_OK)
	{
		//SEGGER_RTT_printf(0, "I2C Read ERROR\n");
		I2CError(&hi2c1);
		//osMutexRelease(I2C1Mutex);
		return -1;
	}

	
	//if(osSemaphoreWait(I2c1RxComplete,100) != osOK) HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
	//osDelay(1);
	//osMutexRelease(I2C1Mutex);
	return 0;
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{	
		osSemaphoreRelease(I2c1RxComplete);
	}
	
}


void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
	if(hi2c->Instance == I2C1)
	{
		osSemaphoreRelease(I2c1TxComplete);
		//HAL_I2C_ErrorCallback
		HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);
	}
	
}
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	if(hi2c->Instance == I2C1)
	{
		//HAL_I2C_DeInit(hi2c);
		//MX_I2C1_Init();
		
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9)|| !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
	  {

		  
		  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_SET);
		  while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_9))
		  {	
			  //HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_8);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
			  DWT_Delay_us(10);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
			  DWT_Delay_us(10);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
		  DWT_Delay_us(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
		  DWT_Delay_us(10);
		  //HAL_GPIO_DeInit(GPIOB,GPIO_PIN_9);
		  //HAL_GPIO_DeInit(GPIOB,GPIO_PIN_8);
		  //GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9;
//		  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
//		  GPIO_InitStruct.Pull = GPIO_PULLUP;
//		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//		  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
//		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  
		  //hi2c->Instance->CR1 = I2C_CR1_SWRST;
		  //DWT_Delay_us(10);
		  //hi2c->Instance->CR1= 0; 
 
	  }
	  HAL_I2C_DeInit(hi2c);
		MX_I2C1_Init(); 
		HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);			
	}
	else if(hi2c->Instance == I2C2)
	{
	  if(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_10)|| !HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11))
	  {		  
		  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
		  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
		  GPIO_InitStruct.Pull = GPIO_PULLUP;
		  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_SET);
		  while(!HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11))
		  {	
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_RESET);
			  DWT_Delay_us(10);
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, GPIO_PIN_SET);
			  DWT_Delay_us(10);
		  }
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_RESET);
		  DWT_Delay_us(10);
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, GPIO_PIN_SET);
		  DWT_Delay_us(10);

 
	  }
	  HAL_I2C_DeInit(hi2c);
		MX_I2C2_Init(); 
		HAL_GPIO_TogglePin(D3_GPIO_Port,D3_Pin);			
	}
}

/* USER CODE END 1 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
