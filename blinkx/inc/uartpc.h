/*********************************************************/
/***  Project:       AraController                     ***/
/***  File:          uartpc.h                          ***/
/***  Created on:    22.07.2016                        ***/
/***  Author:        Andreas Stärker                   ***/
/***  Company:       Blink AG                          ***/
/***  Changes:       23.03.2018 TX_QUEUE implemented   ***/
/*********************************************************/

#ifndef UARTPC_H
#define UARTPC_H

#include "defines.h"

#define UART_TX_QUEUE_SIZE		10

/* USART */
#define PC_USART			         	USART1
#define PC_USART_RCC			        RCC_APB2Periph_USART1
#define PC_USART_AF			        	GPIO_AF_USART1
#define PC_USART_IRQ_CHANNEL			USART1_IRQn
/* TX Pin */
#define PC_USART_TX_RCC			        RCC_AHB1Periph_GPIOA
#define PC_USART_TX_PORT				GPIOA
#define PC_USART_TX_PIN				    GPIO_Pin_9
#define PC_USART_TX_PIN_SOURCE			GPIO_PinSource9
/* RX Pin */
#define PC_USART_RX_RCC			        RCC_AHB1Periph_GPIOA
#define PC_USART_RX_PORT				GPIOA
#define PC_USART_RX_PIN				    GPIO_Pin_10
#define PC_USART_RX_PIN_SOURCE			GPIO_PinSource10


typedef struct
{
	uint8_t in;// position of incoming message
    uint8_t out;//position of outgoing message
    uint16_t len[UART_TX_QUEUE_SIZE];//length of string
    char buffer[UART_TX_QUEUE_SIZE][TX_BUFFER_SIZE];
}TypeUartTxQueue;


//variables for access to interrupt handler
extern char UartPc_TX_Buffer[TX_BUFFER_SIZE];//buffer sent to UART in ISR
//extern unsigned char UartPc_TE_Buffer[TX_BUFFER_SIZE];//buffer filled by send commands
extern char UartPc_RX_Buffer[RX_BUFFER_SIZE];//buffer filled in ISR
extern char UartPc_RE_Buffer[RX_BUFFER_SIZE];//buffer fetched by main()
extern volatile unsigned char UartPc_TX_Pos;
extern volatile unsigned char UartPc_TX_Len;
extern volatile unsigned char UartPc_RX_Pos;
extern unsigned char UartPc_RE_Len;
//extern unsigned char UartPc_TE_Len;



//functions:
void UartPcInit(void);
//void UartPcSend(char * buf, unsigned char len); obsolete
unsigned char UartPcReceive(char * buf, unsigned char maxlen);//called from main to get data
void UartPcWorkTx(void);//cyclic called from main
void UartPcAppendMessageToTxQueue(char * buf, unsigned char len);
void UartPcSendMessageFromTxQueue(void);


#endif

