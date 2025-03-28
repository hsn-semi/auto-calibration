/*********************************************************/
/***  Project:       AraController                     ***/
/***  File:          uartpc.c                          ***/
/***  Created on:    22.07.2016                        ***/
/***  Author:        Andreas Stärker                   ***/
/***  Company:       Blink AG                          ***/
/***  Changes:       29.01.2018: changed to UART1      ***/
/***                 23.03.2018 TX_QUEUE implemented   ***/
/*********************************************************/



#include <stdio.h>
#include <string.h>

#include "stm32f4xx_usart.h"

#include "defines.h"

#include "uartpc.h"


//variables:
char UartPc_TX_Buffer[TX_BUFFER_SIZE];//buffer sent to UART in ISR
char UartPc_TE_Buffer[TX_BUFFER_SIZE];//buffer filled by send commands
char UartPc_RX_Buffer[RX_BUFFER_SIZE];//buffer filled in ISR
char UartPc_RE_Buffer[RX_BUFFER_SIZE];//buffer fetched by main()
volatile unsigned char UartPc_TX_Pos = 0;
volatile unsigned char UartPc_TX_Len = 0;
volatile unsigned char UartPc_RX_Pos = 0;
unsigned char UartPc_RE_Len = 0;
unsigned char UartPc_TE_Len = 0;

TypeUartTxQueue UartTxQueue;



// ********************************************** //
// ***       Function: UartPcInit             *** //
// ***       Purpose: init USART              *** //
// ***       Input: none                      *** //
// ***       Return: none                     *** //
// ********************************************** //
void UartPcInit(void)
{

	  GPIO_InitTypeDef 		GPIO_InitStructure;
	  USART_InitTypeDef 	USART_InitStructure;
	  NVIC_InitTypeDef 		NVIC_InitStructure;


	  /* USART clock enable !! APB2 for USART1 !!*/
	  RCC_APB2PeriphClockCmd(PC_USART_RCC, ENABLE);

	  /* GPIO clock enable */
	  RCC_AHB1PeriphClockCmd(PC_USART_TX_RCC | PC_USART_TX_RCC, ENABLE);

	  /* GPIOG Configuration: USART RXD, TXD */
	  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;//GPIO_Speed_50MHz;
	  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP ;
	  GPIO_InitStructure.GPIO_Pin = PC_USART_RX_PIN;
	  GPIO_Init(PC_USART_RX_PORT, &GPIO_InitStructure);
	  GPIO_InitStructure.GPIO_Pin = PC_USART_TX_PIN;
	  GPIO_Init(PC_USART_TX_PORT, &GPIO_InitStructure);

	  /* connect pins to alternate function USART */
	  GPIO_PinAFConfig(PC_USART_TX_PORT, PC_USART_TX_PIN_SOURCE, PC_USART_AF);
	  GPIO_PinAFConfig(PC_USART_RX_PORT, PC_USART_RX_PIN_SOURCE, PC_USART_AF);

	  USART_DeInit(PC_USART);
	  USART_StructInit(&USART_InitStructure);
	  USART_InitStructure.USART_BaudRate = 61500;//57600 //keine Ahnung warum die tatsächliche baudrate  zu tief ist???
	  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	  USART_InitStructure.USART_Parity = USART_Parity_No;
	  USART_InitStructure.USART_StopBits = USART_StopBits_1;
	  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	  USART_Init(PC_USART, &USART_InitStructure);

	  /* Enable the USART */
	  USART_Cmd(PC_USART, ENABLE);

	  /* Enable the USART6 global Interrupts */
	  USART_ITConfig(PC_USART, USART_IT_RXNE, ENABLE);

	  /* Enable the USART6 global Interrupt */
	  NVIC_InitStructure.NVIC_IRQChannel = PC_USART_IRQ_CHANNEL;
	  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 5;
	  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	  NVIC_Init(&NVIC_InitStructure);


      memset(UartPc_TX_Buffer, 0, sizeof(UartPc_TX_Buffer));
      memset(UartPc_RX_Buffer, 0, sizeof(UartPc_RX_Buffer));
      UartPc_RE_Len = 0;
      UartPc_RX_Pos = 0;

      memset(&UartTxQueue, 0, sizeof(UartTxQueue));

}



// ************************************************** //
// ***       Function: UartPcReceive              *** //
// ***       Purpose: get data from RX buffer     *** //
// ***        and clear RX buffer afterwards      *** //
// ***                                            *** //
// ***       Input: buf - pointer to buffer       *** //
// ***       Input: maxlen - size of buffer       *** //
// ***       Return: lenght of actual data        *** //
// ************************************************** //
unsigned char UartPcReceive(char * buf, unsigned char maxlen)
{
      unsigned char ret = 0; 
      
      
  
      if(UartPc_RE_Len == 0)
      {
            //no data available
            return 0;
      }  
      
      if(maxlen > UartPc_RE_Len)//enough space in buffer
      {
           ret = UartPc_RE_Len;
           memcpy(buf, UartPc_RE_Buffer, UartPc_RE_Len);
           buf[UartPc_RE_Len] = 0;//end sign
           UartPc_RE_Len = 0;           
           return ret;            
      }
      
      
      return 0;
  
}



// ******************************************************************* //
// ***       Function: UartPcWorkTx                                *** //
// ***       Purpose: work funktion for serial interface to PC     *** //
// ***                                                             *** //
// ***                                                             *** //
// ***       Input: none                                           *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void UartPcWorkTx(void)
{
       
       //if no TX interrupt pending
       if(UartPc_TX_Len == 0)
       {

    	   /* sent the next item from the queue */
    	   UartPcSendMessageFromTxQueue();

       }
       
  
      
}


// ******************************************************************* //
// ***       Function: UartPcAppendMessageToTxQueue                *** //
// ***       Purpose: appends data to UART Send Queue              *** //
// ***       Input: buf - string to send                           *** //
// ***              len - length of string                         *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void UartPcAppendMessageToTxQueue(char * buf, unsigned char len)
{


	/* prevent overrun */
	if(UartTxQueue.in >= UART_TX_QUEUE_SIZE)
	{
		UartTxQueue.in = 0;
	}

	//send only, if something to send
	 if(len > 0)
	 {
			//send only, if enough space in the buffer
			if((len + 1) <= TX_BUFFER_SIZE)
			{

				memcpy(UartTxQueue.buffer[UartTxQueue.in], buf, len);
				//append CR
				UartTxQueue.buffer[UartTxQueue.in][len] = 13;
				//update the length
				UartTxQueue.len[UartTxQueue.in] = len + 1;


				UartTxQueue.in++;

			}

	 }



	return;

}

// ******************************************************************* //
// ***       Function: UartPcSendMessageFromTxQueue                *** //
// ***       Purpose: appends data to UART Send Queue              *** //
// ***       Input: none                                           *** //
// ***       Return: none                                          *** //
// ******************************************************************* //
void UartPcSendMessageFromTxQueue(void)
{

	/* check, if message to send in Queue available */
	if(UartTxQueue.in != UartTxQueue.out)
	{

		  /* copy the content to the TX-Buffer */
		  //start transmission
		  memcpy(UartPc_TX_Buffer, UartTxQueue.buffer[UartTxQueue.out], UartTxQueue.len[UartTxQueue.out]);
		  //update the length of the transmit output buffer
		  UartPc_TX_Len = UartTxQueue.len[UartTxQueue.out];

		  //start transmission by sending the first byte
		  //USART->DR = UartPc_TX_Buffer[0];
		  USART_SendData(PC_USART, UartPc_TX_Buffer[0]);
		  UartPc_TX_Pos = 1;

		  /* Enable the USART transmit Interrupt */
		  USART_ITConfig(PC_USART, USART_IT_TXE, ENABLE);

		  /* increase output pointer */
		  UartTxQueue.out++;

		  /* prevent overrun of output pointer */
		  if(UartTxQueue.out >= UART_TX_QUEUE_SIZE)
		  {
			  UartTxQueue.out = 0;
		  }

		  /* reset pointers, if last message had been sent */
		  if(UartTxQueue.in == UartTxQueue.out)
		  {
			  UartTxQueue.in = 0;
			  UartTxQueue.out = 0;
		  }

	}


	return;

}


