/**
  ******************************************************************************
  * @file    stm32f1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f1xx_hal.h"
#include "stm32f1xx.h"
#include "stm32f1xx_it.h"

/* USER CODE BEGIN 0 */

#define BUFFERSIZE_right 20
extern uint8_t recv_end_flag_right,Rx_len_right;

#define BUFFERSIZE_left 20
extern uint8_t recv_end_flag_left,Rx_len_left;
extern uint8_t main_sta;

uint16_t USART_RX_STA=0;   //接收状态标记	

uint8_t serial_record[10];
uint8_t serial_buffer[10];
uint8_t USART_RX_BUF[USART_REC_LEN]; 

#define BUFFERSIZE 20
extern uint8_t recv_end_flag;
extern uint8_t Rx_len;


union recieveData  //接收到的数据
{
	float d;    //左右轮速度
	unsigned char data[4];
}leftdata,rightdata;       //接收的左右轮数据

float odometry_right=0,odometry_left=0;//串口得到的左右轮速度
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/*            Cortex-M3 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles DMA1 channel5 global interrupt.
*/
void DMA1_Channel5_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */

  /* USER CODE END DMA1_Channel5_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */

  /* USER CODE END DMA1_Channel5_IRQn 1 */
}

/**
* @brief This function handles DMA1 channel6 global interrupt.
*/
void DMA1_Channel6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */

  /* USER CODE END DMA1_Channel6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_rx);
  /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */

  /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
* @brief This function handles TIM1 update interrupt.
*/
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles USART1 global interrupt.
*/
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  	uint32_t temp;
	/*	如果是串口1中断	*/
	if(USART1 == huart1.Instance)
	{	/* 如果是串口1IDLE中断	*/
		if(RESET != __HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE)){
			/*	清除中断标志	*/
			__HAL_UART_CLEAR_IDLEFLAG(&huart1);
			/*	停止DMA接收	*/
			HAL_UART_DMAStop(&huart1);
			/*	获取DMA当前还有多少未填充	*/
			temp  = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx);
			/*	计算串口接收到的数据个数	*/
			Rx_len_right =  BUFFERSIZE_right - temp; 
			recv_end_flag_right = 1;
		}
	}
  /* USER CODE END USART1_IRQn 1 */
}

/**
* @brief This function handles USART2 global interrupt.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
	
  /* USER CODE BEGIN USART2_IRQn 1 */
					uint32_t temp;
	/*	如果是串口1中断	*/
	if(USART2 == huart2.Instance)
	{	/* 如果是串口1IDLE中断	*/
		if(RESET != __HAL_UART_GET_FLAG(&huart2,UART_FLAG_IDLE)){
			/*	清除中断标志	*/
			__HAL_UART_CLEAR_IDLEFLAG(&huart2);
			/*	停止DMA接收	*/
			HAL_UART_DMAStop(&huart2);
			/*	获取DMA当前还有多少未填充	*/
			temp  = __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
			/*	计算串口接收到的数据个数	*/
			Rx_len_left =  BUFFERSIZE_left - temp; 
			recv_end_flag_left = 1;
		}
	}
  /* USER CODE END USART2_IRQn 1 */
}

/**
* @brief This function handles USART3 global interrupt.
*/
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
	if(USART3 == huart3.Instance)
	{
	
		HAL_UART_Receive_IT(&huart3,(uint8_t*)serial_record,10);	
		
		if(serial_record[8]==0x0d&&serial_record[9]==0x0a)
		{
			for(int i=0;i<10;i++)
			{	
				serial_buffer[i]=serial_record[i];
				
					for(int t=0;t<4;t++)
            {
                rightdata.data[t]=serial_buffer[t];
                leftdata.data[t]=serial_buffer[t+4];
            }
						 odometry_right=rightdata.d;//convert to int 单位mm/s
             odometry_left=leftdata.d;//convert to int  单位mm/s
				
			}
			for(int i=0;i<10;i++)//数组清零，准备接收下一组数据
			{
				serial_record[i]=0;			
			}
		}
		

		
}

}	
			



  /* USER CODE END USART3_IRQn 1 */


/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
