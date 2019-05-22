/**
  ******************************************************************************
  * @file    GPIO/GPIO_EXTI/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
#include "./key/bsp_exti.h" 
#include "./usart/bsp_debug_usart.h"

#include "los_sys.h"
#include "los_task.ph"
#include "los_queue.h"
#include "los_sem.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup GPIO_EXTI
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
//void SysTick_Handler(void)
//{
//  HAL_IncTick();
//}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

uint32_t wi = 0;
/* 定义消息队列的句柄 */
extern UINT32 Test_Queue_Handle;
/* 定义二值信号量的句柄 */
extern UINT32 BinarySem_Handle;

extern char Usart_Rx_Buf[100];

static uint32_t send_data1 = 1;
static uint32_t send_data2 = 2;


void KEY1_IRQHandler(void)
{
	UINTPTR uvIntSave;
	
	uvIntSave = LOS_IntLock();		//关中断
  
  //确保是否产生了EXTI Line中断
	if(__HAL_GPIO_EXTI_GET_IT(KEY1_INT_GPIO_PIN) != RESET) 
	{
    /* 将数据写入（发送）到队列中，等待时间为 0  */
    LOS_QueueWrite(	Test_Queue_Handle,	/* 写入（发送）队列的ID(句柄) */
                    &send_data1,				/* 写入（发送）的数据 */
                    sizeof(send_data1),	/* 数据的长度 */
                    0);									/* 等待时间为 0  */
    //清除中断标志位
		__HAL_GPIO_EXTI_CLEAR_IT(KEY1_INT_GPIO_PIN);     
	}  
  
	LOS_IntRestore(uvIntSave);	//开中断
}

void KEY2_IRQHandler(void)
{
	UINTPTR uvIntSave;
	
	uvIntSave = LOS_IntLock();		//关中断
  
  //确保是否产生了EXTI Line中断
	if(__HAL_GPIO_EXTI_GET_IT(KEY2_INT_GPIO_PIN) != RESET) 
	{
		/* 将数据写入（发送）到队列中，等待时间为 0  */
		LOS_QueueWrite(	Test_Queue_Handle,	/* 写入（发送）队列的ID(句柄) */
										&send_data2,				/* 写入（发送）的数据 */
										sizeof(send_data2),	/* 数据的长度 */
										0);									/* 等待时间为 0  */
    //清除中断标志位
		__HAL_GPIO_EXTI_CLEAR_IT(KEY2_INT_GPIO_PIN);     
	}  
  
	LOS_IntRestore(uvIntSave);	//开中断
}


void  DEBUG_USART_IRQHandler(void)
{
	UINTPTR uvIntSave;
	
	uvIntSave = LOS_IntLock();		//关中断
  
	if(__HAL_UART_GET_FLAG( &UartHandle, UART_FLAG_RXNE ) != RESET)
	{		
    
    Usart_Rx_Buf[wi++] = (uint8_t)READ_REG(UartHandle.Instance->DR);
    if (wi >= USART_RX_LEN)wi = 0;
	}
  else if(__HAL_UART_GET_FLAG(&UartHandle, UART_FLAG_IDLE) != RESET)
  {
    __HAL_UART_CLEAR_IDLEFLAG(&UartHandle);
    
    //给出二值信号量 ，发送接收到新数据标志，供前台程序查询
    LOS_SemPost( BinarySem_Handle ); //给出二值信号量 xSemaphore
    wi = 0;
  }
  
	LOS_IntRestore(uvIntSave);	//开中断
}




/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
