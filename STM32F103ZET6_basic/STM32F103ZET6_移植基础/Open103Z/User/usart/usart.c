#include "usart.h"

/******************************************************************************/

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef NVIC_InitStructure; 
	/* Configure the NVIC Preemption Priority Bits */  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
	
	/* Enable the USARTy Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART_IRQ;	 
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

void USART_init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	
	/* config USART clock */
	if (TEST_USART == USART1)
	{
		RCC_APB2PeriphClockCmd(TEST_USART_CLS, ENABLE);
	}
	else
	{													
		RCC_APB1PeriphClockCmd(TEST_USART_CLS, ENABLE);
	}
	RCC_APB2PeriphClockCmd(TEST_USART_TXD_CLK|TEST_USART_RXD_CLK, ENABLE);
	
	
	/* USART GPIO config */
	/* Configure USART Tx as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = TEST_USART_TXD_PIN;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(TEST_USART_TXD_GPIO_PORT, &GPIO_InitStructure);    
	/* Configure USART Rx as input floating */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	
	USART_InitStructure.USART_BaudRate = TEST_USART_BAUD;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(TEST_USART, &USART_InitStructure);
	
	USART_ITConfig(TEST_USART, USART_IT_RXNE, ENABLE);
	
	USART_Cmd(TEST_USART, ENABLE);
	
	NVIC_Configuration();
}

/// �ض���c�⺯��printf��USART
int fputc(int ch, FILE *f)
{
		/* ����һ���ֽ����ݵ�USART1 */
		USART_SendData(TEST_USART, (uint8_t) ch);
		
		/* �ȴ�������� */
		while (USART_GetFlagStatus(TEST_USART, USART_FLAG_TC) == RESET);		
	
		return (ch);
}

/// �ض���c�⺯��scanf��USART
int fgetc(FILE *f)
{
		/* �ȴ�����1�������� */
		while (USART_GetFlagStatus(TEST_USART, USART_FLAG_RXNE) == RESET);

		return (int)USART_ReceiveData(TEST_USART);
}

/*********************************************END OF FILE**********************/
