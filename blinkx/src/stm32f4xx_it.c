/**
  ******************************************************************************
  * @file    TIM_PWM_Output/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    19-September-2011
  * @brief   Main Interrupt Service Routines.
  *         This file provides template for all exceptions handler and
  *         peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h2>
  ******************************************************************************
  */ 
/* Changes: AST 23.03.2018 USART TXE Bugfix */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "defines.h"

#include <stdio.h>
#include "stdlib.h"
#include "string.h"

#include "uartpc.h"
#include "stepmotor.h"



#include "main.h"


/** @addtogroup STM32F4_Discovery_Peripheral_Examples
  * @{
  */

/** @addtogroup TIM_PWM_Output
  * @{
  */ 

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


//extern TypeServoCurrent ServoCurrent;



//uint16_t PhaseU, PhaseV, PhaseW;//phases
//uint32_t MagU, MagV, MagW;//magnitudes

unsigned char GoTimeBase = 0;
uint16_t GoTimeBaseCounter = 0;

uint32_t DelayTic;
uint16_t DelayCounter = 0;

uint16_t Go10msCounter = 0;
uint8_t  Go10ms = 0;
uint8_t  Go1ms = 0;


//extern unsigned char AverageReadyServo;



//volatile unsigned char PhaseCompleteU;
//volatile unsigned char PhaseCompleteV;
//volatile unsigned char PhaseCompleteW;

//volatile uint16_t CurrentRegulationTimeoutU = 0;
//volatile uint16_t CurrentRegulationTimeoutV = 0;
//volatile uint16_t CurrentRegulationTimeoutW = 0;
//#define CURENT_REGULATION_TIMEOUT  200 //in 50us: 200 = 10ms



volatile uint32_t EncoderCount = 0;
volatile uint8_t EncoderSync = 0;
#define ENCODER_COUNT_LOW_LIMIT		5000//
#define ENCODER_COUNT_HIGH_LIMIT	40000//larger than this: motor is stopped
#define ENCODER_COUNT_INTERVAL_MIN  2 //depending on Encoder
volatile uint16_t EncoderCountInterval = ENCODER_COUNT_INTERVAL_MIN;//depending on Speed
uint16_t CountInterval = 0;
#define ENCODER_CONSTANT		1200000// 60 s/min * 20kHz
uint32_t EncoderConstant = ENCODER_CONSTANT;



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
  {}
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
  {}
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
  {}
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
  {}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{}

/**
  * @brief  This function handles PendSV_Handler exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 

/**
  * @}
  */ 

/******************************************************************************/
/*                       Timer 9 Interrupt-Handler                            */
/*                                                                            */
/******************************************************************************/
void TIM1_BRK_TIM9_IRQHandler(void)
{
	//Timer update interrupt
	if(TIM_GetITStatus(TIM9, TIM_IT_Update) == SET)
	{

		/* Clear TIM update interrupt pending bit */
		TIM_ClearITPendingBit(TIM9, TIM_IT_Update);

	}


}

/******************************************************************************/
/*                       Timer 1 and Timer 10 Interrupt-Handler               */
/*                              for Stepmotor X                               */
/******************************************************************************/
void TIM1_UP_TIM10_IRQHandler(void)
{


	//Timer update interrupt
	if(TIM_GetITStatus(TIM10, TIM_IT_Update) == SET)
	{

		if(GPIO_ReadInputDataBit(StepmotorHW[MOTOR_X].stepGPIO.port, StepmotorHW[MOTOR_X].stepGPIO.pin))
		{
			GPIO_ResetBits(StepmotorHW[MOTOR_X].stepGPIO.port, StepmotorHW[MOTOR_X].stepGPIO.pin);
			StepmotorControl[MOTOR_X].counter++;

			StepmotorControl[MOTOR_X].absolutecounter += StepmotorControl[MOTOR_X].direction;
		}
		else
		{
			GPIO_SetBits(StepmotorHW[MOTOR_X].stepGPIO.port, StepmotorHW[MOTOR_X].stepGPIO.pin);
		}

		if(StepmotorControl[MOTOR_X].counter >= StepmotorControl[MOTOR_X].stepstogo)
		{
			/* disable Timer */
			TIM_Cmd(TIM10, DISABLE);

			StepmotorControl[MOTOR_X].finished = 1;

		}


		/* Clear TIM update interrupt pending bit */
		TIM_ClearITPendingBit(TIM10, TIM_IT_Update);
	}

	//Timer update interrupt
		if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
		{
			/* Clear TIM update interrupt pending bit */
			TIM_ClearITPendingBit(TIM1, TIM_IT_Update);

		}


}

/******************************************************************************/
/*                       Timer 11 Interrupt-Handler                           */
/*                         for   Stepmotor Y                                   */
/******************************************************************************/
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{

	//Timer update interrupt
		if(TIM_GetITStatus(TIM11, TIM_IT_Update) == SET)
		{

			if(GPIO_ReadInputDataBit(StepmotorHW[MOTOR_Y].stepGPIO.port, StepmotorHW[MOTOR_Y].stepGPIO.pin))
			{
				GPIO_ResetBits(StepmotorHW[MOTOR_Y].stepGPIO.port, StepmotorHW[MOTOR_Y].stepGPIO.pin);
				StepmotorControl[MOTOR_Y].counter++;

				StepmotorControl[MOTOR_Y].absolutecounter += StepmotorControl[MOTOR_Y].direction;
			}
			else
			{
				GPIO_SetBits(StepmotorHW[MOTOR_Y].stepGPIO.port, StepmotorHW[MOTOR_Y].stepGPIO.pin);
			}

			if(StepmotorControl[MOTOR_Y].counter >= StepmotorControl[MOTOR_Y].stepstogo)
			{
				/* disable Timer */
				TIM_Cmd(TIM11, DISABLE);

				StepmotorControl[MOTOR_Y].finished = 1;

			}


			/* Clear TIM update interrupt pending bit */
			TIM_ClearITPendingBit(TIM11, TIM_IT_Update);
		}

}

/******************************************************************************/
/*                       Timer 12 Interrupt-Handler                           */
/*                      for Stepmotor Z                                       */
/******************************************************************************/
void TIM8_BRK_TIM12_IRQHandler(void)
{



	//Timer 12 update interrupt for DC-Motor speed measuring
	if(TIM_GetITStatus(TIM12, TIM_IT_Update) == SET)
	{




			/* Clear TIM update interrupt pending bit */
			TIM_ClearITPendingBit(TIM12, TIM_IT_Update);


	}





}

/******************************************************************************/
/*                       Timer 13 Interrupt-Handler                           */
/*                      for Speed COunting DC-Motor                           */
/******************************************************************************/
void TIM8_UP_TIM13_IRQHandler(void)
{


	//Timer update interrupt
	if(TIM_GetITStatus(TIM13, TIM_IT_Update) == SET)
	{


		/* Clear TIM update interrupt pending bit */
		TIM_ClearITPendingBit(TIM13, TIM_IT_Update);
	}




}

/******************************************************************************/
/*                       Timer 4 Interrupt-Handler                            */
/*                      for Audio                                             */
/******************************************************************************/
void TIM4_IRQHandler(void)
{

	//Timer update interrupt
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) == SET)
	{




		/* Clear TIM update interrupt pending bit */
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}

}


/******************************************************************************/
/*                       Timer 6 Interrupt-Handler                            */
/*                      Audio Timer                                           */
/******************************************************************************/
void TIM6_DAC_IRQHandler(void)
{




	//Timer update interrupt
	if(TIM_GetITStatus(TIM6, TIM_IT_Update) == SET)
	{



			/* Clear TIM update interrupt pending bit */
			TIM_ClearITPendingBit(TIM6, TIM_IT_Update);
	}


}

/******************************************************************************/
/*                       Timer 7 Interrupt-Handler                            */
/*                       System Timer                                         */
/******************************************************************************/
void TIM7_IRQHandler(void)
{

	//Timer update interrupt
	if(TIM_GetITStatus(TIM7, TIM_IT_Update) == SET)
	{

		//debug red LED
		//LedRed(ON);


		GoTimeBaseCounter++;
		if(GoTimeBaseCounter >= 200)//every 100ms
		{
			GoTimeBaseCounter = 0;
			GoTimeBase = 1;
			SystemTimeIn100ms++;
		}

		DelayCounter++;
		if(DelayCounter >= 2)//every 1ms
		{
				DelayCounter = 0;
				Go1ms = 1;
				DelayTic++;
		}

		Go10msCounter++;
		if(Go10msCounter >= 20)//every 10ms
		{
			Go10msCounter = 0;
			Go10ms = 1;
		}

		//debug red LED
		//LedRed(OFF);

		/* Clear TIM update interrupt pending bit */
		TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
	}

}


void TIM8_TRG_COM_TIM14_IRQHandler(void)
{
	//Timer update interrupt
		if(TIM_GetITStatus(TIM14, TIM_IT_Update) == SET)
		{


			/* Clear TIM update interrupt pending bit */
			TIM_ClearITPendingBit(TIM14, TIM_IT_Update);
		}

}


void ADC_IRQHandler(void)
{


	if(ADC_GetITStatus(ADC1, ADC_IT_EOC) == SET)
	{


		/* Clear ADC1  interrupt pending bit */
		ADC_ClearITPendingBit(ADC1, ADC_IT_EOC);

		/* debug */
		//LedGreen(OFF);

	}

	if(ADC_GetITStatus(ADC2, ADC_IT_EOC) == SET)
	{


		/* Clear ADC2  interrupt pending bit */
		ADC_ClearITPendingBit(ADC2, ADC_IT_EOC);

		/* debug */
		//LedBlue(OFF);
	}

	if(ADC_GetITStatus(ADC3, ADC_IT_EOC) == SET)
	{

		/* Clear ADC1  interrupt pending bit */
		ADC_ClearITPendingBit(ADC3, ADC_IT_EOC);
	}

}

void EXTI0_IRQHandler(void)
{
		if(EXTI_GetITStatus(EXTI_Line0) != RESET)
		{



				/* Clear the EXTI line 0 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line0);
		}
}

void EXTI1_IRQHandler(void)
{



		if(EXTI_GetITStatus(EXTI_Line1) != RESET)
		{



				/* Clear the EXTI line 1 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line1);
		}
}

/******************************************************************************/
/*              External Interrupt Line 3 Interrupt-Handler                   */
/*                      for Encoder                                           */
/******************************************************************************/

void EXTI3_IRQHandler(void)
{


		/* Axis Y EncB */
		if(EXTI_GetITStatus(EXTI_Line3) != RESET)
		{




				/* Clear the EXTI line 11 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line3);
		}

}

/******************************************************************************/
/*              External Interrupt Line 5 .. 9 Interrupt-Handler              */
/*                   Line 7:  for Encoder Stepmotor Z                         */
/*                   Line 9:  for Encoder Stepmotor Y                         */
/******************************************************************************/

void EXTI9_5_IRQHandler(void)
{

	    int8_t levb;
		int8_t leva;

		/* Encoder Filter Changer */
		if(EXTI_GetITStatus(EXTI_Line5) != RESET)
		{

			//optional delay
			asm("nop");
			asm("nop");
			leva = GPIO_ReadInputDataBit(STEPMOTOR_X_ENCA_GPIO_PORT, STEPMOTOR_X_ENCA_PIN);
			levb = GPIO_ReadInputDataBit(STEPMOTOR_X_ENCB_GPIO_PORT, STEPMOTOR_X_ENCB_PIN);
			if(levb == StepmotorControl[MOTOR_X].oldb)
			{
				//no change -> exit
				/* Clear the EXTI line 11 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line5);
				//LedYellow(OFF);
				return;
			}
			//remember the level
			StepmotorControl[MOTOR_X].oldb = levb;
			if(levb == 1)//rising edge: do the counting
			{
				if(leva != StepmotorControl[MOTOR_X].olda)
				{
					if(leva == STEPMOTOR_X_ENCODER_DIRECTION)
					{
						StepmotorControl[MOTOR_X].encoderposition++;
					}
					else
					{
						StepmotorControl[MOTOR_X].encoderposition--;
					}
				}
			}
			else//falling Edge: remember the Level a
			{
				/* remember the Level B */
				StepmotorControl[MOTOR_X].olda = leva;
			}


			/* Clear the EXTI line 5 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line5);

		}

		/* Encoder optic Disc */
		if(EXTI_GetITStatus(EXTI_Line7) != RESET)
		{

			//optional delay
			asm("nop");
			asm("nop");
			leva = GPIO_ReadInputDataBit(STEPMOTOR_Y_ENCA_GPIO_PORT, STEPMOTOR_Y_ENCA_PIN);
			levb = GPIO_ReadInputDataBit(STEPMOTOR_Y_ENCB_GPIO_PORT, STEPMOTOR_Y_ENCB_PIN);
			if(levb == StepmotorControl[MOTOR_Y].oldb)
			{
				//no change -> exit
				/* Clear the EXTI line 11 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line7);
				//LedYellow(OFF);
				return;
			}
			//remember the level
			StepmotorControl[MOTOR_Y].oldb = levb;
			if(levb == 1)//rising edge: do the counting
			{
				if(leva != StepmotorControl[MOTOR_Y].olda)
				{
					if(leva == STEPMOTOR_Y_ENCODER_DIRECTION)
					{
						StepmotorControl[MOTOR_Y].encoderposition++;
					}
					else
					{
						StepmotorControl[MOTOR_Y].encoderposition--;
					}
				}
			}
			else//falling Edge: remember the Level a
			{
				/* remember the Level B */
				StepmotorControl[MOTOR_Y].olda = leva;
			}


			/* Clear the EXTI line 5 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line7);

		}

		/*  */
		if(EXTI_GetITStatus(EXTI_Line8) != RESET)
		{



				/* Clear the EXTI line 5 pending bit */
				EXTI_ClearITPendingBit(EXTI_Line8);
		}

		/* Stepmotor Y */
		if(EXTI_GetITStatus(EXTI_Line9) != RESET)
		{


			/* Clear the EXTI line 11 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line9);
		}
}


/******************************************************************************/
/*              External Interrupt Line 10 .. 15 Interrupt-Handler            */
/*                      for Encoder Stepmotor X                               */
/******************************************************************************/


void EXTI15_10_IRQHandler(void)
{



	/* Stepmotor Enc B */
	if(EXTI_GetITStatus(EXTI_Line11) != RESET)
	{



			/* Clear the EXTI line 11 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line11);
	}

	/* Axis 2 EncB */
	if(EXTI_GetITStatus(EXTI_Line13) != RESET)
	{

			/* Clear the EXTI line 11 pending bit */
			EXTI_ClearITPendingBit(EXTI_Line13);
	}

}

/******************************************************************************/
/*                    SPI2 Interrupt-Handler                                  */
/*                      for Inclinometer                                      */
/******************************************************************************/

void SPI2_IRQHandler(void)
{
	//Transmit Interrupt
	if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_TXE) != RESET)
	{


			/* Clear the INT */
			SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_TXE);
	}

	//Receive interrupt
	if(SPI_I2S_GetITStatus(SPI2, SPI_I2S_IT_RXNE) != RESET)
	{

		/* Clear the INT */
		SPI_I2S_ClearITPendingBit(SPI2, SPI_I2S_IT_RXNE);
	}
}





void SPI3_IRQHandler(void)
{
		//Transmit Interrupt
		if(SPI_I2S_GetITStatus(SPI3, SPI_I2S_IT_TXE) != RESET)
		{

				/* Clear the INT */
				SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_TXE);
		}

		//Receive interrupt
		if(SPI_I2S_GetITStatus(SPI3, SPI_I2S_IT_RXNE) != RESET)
		{

			/* Clear the INT */
			SPI_I2S_ClearITPendingBit(SPI3, SPI_I2S_IT_RXNE);
		}
}


/******************************************************************************/
/*                       USART 1 Interrupt-Handler                            */
/*                        for Communication with PC                           */
/******************************************************************************/

void USART1_IRQHandler(void)
{

	    uint16_t dat;
		unsigned char len;

		//Transmit Interrupt
		if(USART_GetITStatus(USART1, USART_IT_TXE) == SET)
		{
				/* ToDo */
				 len = UartPc_TX_Len;

				 if(len == UartPc_TX_Pos)//last byte just sent
				 {
							//reset pointers
					 UartPc_TX_Len = 0;
					 UartPc_TX_Pos = 0;

							/* Disable the USART1 transmit Interrupt */
							USART_ITConfig(USART1, USART_IT_TXE, DISABLE);
							return;
				 }
				 if(len > UartPc_TX_Pos)//still bytes to transmit
				 {
					 //USART1->DR = UartPc_TX_Buffer[UartPc_TX_Pos];
					 USART_SendData(USART1, UartPc_TX_Buffer[UartPc_TX_Pos]);
					 UartPc_TX_Pos++;
				 }

				 /* Clear the INT */
				 USART_ClearITPendingBit(USART1, USART_IT_TXE);
		}
		//Receive interrupt
		if(USART_GetITStatus(USART1, USART_IT_RXNE) == SET)
		{
				dat = USART_ReceiveData(USART1);

				 if(dat == 13)//CR
				 {
					   //end sign received:
					   //ChecksumResult = 0;//erase checksum flag, if old data not fetched until now from main(), then they are lost because overwritten
					   //copy data to the receive output the buffer
					   memcpy(UartPc_RE_Buffer,UartPc_RX_Buffer,UartPc_RX_Pos);
					   //update the len
					   UartPc_RE_Len = UartPc_RX_Pos;
					   //reset the len of the receive input buffer
					   UartPc_RX_Pos = 0;

					   //debug
					   //DebugLedOff();

					   return;
				 }

				 //append byte to buffer
				 if(UartPc_RX_Pos <  (RX_BUFFER_SIZE - 1))
				 {
					UartPc_RX_Buffer[UartPc_RX_Pos] = dat;
					UartPc_RX_Pos++;
				 }

				/* Clear the INT */
				USART_ClearITPendingBit(USART1, USART_IT_RXNE);
		}


}

/******************************************************************************/
/*                       USART 2 Interrupt-Handler                            */
/*                                                                             */
/******************************************************************************/

void USART2_IRQHandler(void)
{


	//Transmit Interrupt
	if(USART_GetITStatus(USART2, USART_IT_TXE) == SET)
	{


			/* Clear the INT */
			USART_ClearITPendingBit(USART2, USART_IT_TXE);
	}
	//Receive interrupt
	if(USART_GetITStatus(USART2, USART_IT_RXNE) == SET)
	{


			/* Clear the INT */
			USART_ClearITPendingBit(USART2, USART_IT_RXNE);
	}


}

/******************************************************************************/
/*                       USART 3 Interrupt-Handler                            */
/*                                                                            */
/******************************************************************************/

void USART3_IRQHandler(void)
{




		//Transmit Interrupt
		if(USART_GetITStatus(USART3, USART_IT_TXE) == SET)
		{

				/* Clear the INT */
				USART_ClearITPendingBit(USART3, USART_IT_TXE);
		}
		//Receive interrupt
		if(USART_GetITStatus(USART3, USART_IT_RXNE) == SET)
		{


				/* Clear the INT */
				USART_ClearITPendingBit(USART3, USART_IT_RXNE);
		}




}

/******************************************************************************/
/*                       USART 6 Interrupt-Handler                            */
/*                                                                            */
/******************************************************************************/

void USART6_IRQHandler(void)
{





		//Transmit Interrupt
		if(USART_GetITStatus(USART6, USART_IT_TXE) == SET)
		{



				/* Clear the INT */
				USART_ClearITPendingBit(USART6, USART_IT_TXE);
		}
		//Receive interrupt
		if(USART_GetITStatus(USART6, USART_IT_RXNE) == SET)
		{



				/* Clear the INT */
				USART_ClearITPendingBit(USART6, USART_IT_RXNE);
		}


}

void I2C2_EV_IRQHandler(void)
{
		//Transmit Interrupt
		if(I2C_GetITStatus(I2C2, I2C_IT_EVT) != RESET)
		{
			switch(I2C_GetLastEvent(I2C2))
			{
				case I2C_EVENT_MASTER_MODE_SELECT:
				break;
				case I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED:
				break;
				case I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED:
				break;
				case I2C_EVENT_MASTER_BYTE_TRANSMITTING:
				break;
				case I2C_EVENT_MASTER_BYTE_TRANSMITTED:
				break;
				case I2C_EVENT_MASTER_BYTE_RECEIVED:
				break;

			}

			/* Clear the INT */
			I2C_ClearITPendingBit(I2C2, I2C_IT_EVT);
		}


}

/******************************************************************************/
/*               DMA 1 Stream 6 Interrupt-Handler                             */
/*                   NeoPixel Ring (for debug only)                           */
/******************************************************************************/

void DMA1_Stream6_IRQHandler(void)
{

	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_HTIF6) != RESET)
	{
			//LedGreen(ON);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_HTIF6);

	}

	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TCIF6) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TCIF6);

	}

	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_TEIF6) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_TEIF6);

	}

	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_FEIF6) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_FEIF6);

	}

	if(DMA_GetITStatus(DMA1_Stream6, DMA_IT_DMEIF6) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA1_Stream6, DMA_IT_DMEIF6);

	}


}


/******************************************************************************/
/*               DMA 2 Stream 5 Interrupt-Handler                             */
/*                   NeoPixel Buton (for debug only)                          */
/******************************************************************************/

void DMA2_Stream5_IRQHandler(void)
{

	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_HTIF5) != RESET)
	{
			//LedRed(ON);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_HTIF5);

	}

	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TCIF5) != RESET)
	{
			//LedRed(OFF);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);

	}

	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_TEIF5) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the pending bit */
			DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TEIF5);

	}

	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_FEIF5) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the  pending bit */
			DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_FEIF5);

	}

	if(DMA_GetITStatus(DMA2_Stream5, DMA_IT_DMEIF5) != RESET)
	{
			//LedGreen(OFF);
			/* Clear the  pending bit */
			DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_DMEIF5);

	}


}






/* Delay of n ms */

void Delayms(uint32_t n)
{
	uint32_t start, end;

	start = DelayTic;
	end = DelayTic + n;

	/* overflow detection */
	if(end < start)
	{
		//reset counter
		DelayTic = 0;
		start = 0;
		end = n;
	}

	while(end > DelayTic)
	{
		// DelayTic increases in Timer interrupt handler
	}



}



/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
