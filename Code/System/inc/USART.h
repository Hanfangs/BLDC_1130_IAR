#ifndef  _USART_H
#define  _USART_H


#include "SysConfig.h"
#define BAUDRATE_FRE          115200

#define USART              	 		USART1
#define USART_RCC         	 		RCC_APB2Periph_USART1

#define USART_GPIO_RCC    			RCC_AHBPeriph_GPIOB
#define USART_TX_GPIO_PinSource		        	GPIO_PinSource6
#define USART_RX_GPIO_PinSource		        	GPIO_PinSource7	
#define USART_TX		       		 	GPIO_Pin_6	// out
#define USART_RX		       		 	GPIO_Pin_7	// in 
#define USART_GPIO_PORT    			GPIOB   
#define MAX_LEN_CMD	        		128+8

void USART2_Config(void);
void Usart2_RxEnable(void);
void Usart2_TxEnable(void);
void Usart2_StartRx(void);
void Usart2_StartTx(u8 *pbyData, u16 wLength);
void Usart2_RxEnd(void);
void Usart2_TxEnd(void);
void Usart2_StartTxDelay(void);
void Usart2_TxEndDelay(void);
FlagStatus Is_Usart2_RxEnd(void);
FlagStatus Is_Usart2_TxEnd(void);
u16 Usart2_GetRxLength(void);
void Usart2_GetRxData(u8 *pbyData, u16 wLength);
FlagStatus Com2_Recv(u8 *pbyData, u16 *pwLength);
void Com2_Send(u8 *pbyData, u16 wLength);

FlagStatus Is_Uart2StartTxDelayEnd(void);
void Usart2_ReadForTx(void);
#endif
