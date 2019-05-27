 #include "main.h"
 
u8 uart_rx_step;
u8 f_rx_process;
unsigned char uart_rxbuf[32];
/**
  * @brief  Gets numeric values from the hyperterminal.
  * @param  None
  * @retval None
  */
uint8_t USART_Scanf(uint32_t value)
{
  uint32_t index = 0;
  uint32_t tmp[2] = {0, 0};

//  while (index < 2)
//  {
//    /* Loop until RXNE = 1 */
//    while (USART_GetFlagStatus(EVAL_COM1, USART_FLAG_RXNE) == RESET)
//    {}
//    tmp[index++] = (USART_ReceiveData(EVAL_COM1));
//    if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
//    {
//      printf("\n\r Please enter valid number between 0 and 9 \n\r");
//      index--;
//    }
//  }
//  /* Calculate the Corresponding value */
//  index = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
//  /* Checks */
//  if (index > value)
//  {
//    printf("\n\r Please enter valid number between 0 and %d \n\r", value);
//    return 0xFF;
//  }
  index=10;
  return index;
}
uint16_t receive_data(void)
{
	return USART_ReceiveData(USART1);
}

//void pm25_USART1_RX(void)
//{	static u8 rx_count;
////	u8 temp_reg1;

//	switch(uart_rx_step)				//地址匹配接收
//	{
//		case 0:
//			uart_rxbuf[0] =USART_ReceiveData(USART1);	//读出地址,清除RXNE&&RWU标志,
//			if(uart_rxbuf[0]==0x51)
//			{	rx_count =0;
//				uart_rx_step =1;
//			}
//			else
//				__NOP;
//			break;

//		case 1:
//			uart_rxbuf[1] =USART_ReceiveData(USART1);	//读出地址,清除RXNE&&RWU标志,
//			if(uart_rxbuf[1]==0x4d)
//			{	rx_count =1;
//				uart_rx_step =2;
//			}
//			else
//				uart_rx_step =0;
//			break;

//		case 2:
//			uart_rxbuf[++rx_count] =USART_ReceiveData(USART1);//命令
//			if(rx_count >=31)
//			{	uart_rx_step =0;
//				f_rx_process =1;		//转入后台处理接收到的数据
//			}
//			break;
//	}
//}

void pm25_USART1_RX(void)
{	static u8 rx_count;
//	u8 temp_reg1;

	switch(uart_rx_step)				//地址匹配接收
	{
		case 0:
			uart_rxbuf[0] =USART_ReceiveData(USART1);	//读出地址,清除RXNE&&RWU标志,
			if(uart_rxbuf[0]==0x51)
			{	
				rx_count =0;
				uart_rx_step =1;
			}
			else
				__NOP;
			break;

		case 1:
			uart_rxbuf[1] =USART_ReceiveData(USART1);	//读出地址,清除RXNE&&RWU标志,
			if(uart_rxbuf[1]==0x4d)
			{	
				rx_count =1;
				uart_rx_step =2;
			}
			else
				uart_rx_step =0;
			break;

		case 2:
			uart_rxbuf[++rx_count] =USART_ReceiveData(USART1);//命令
			if(rx_count >=31)
			{	uart_rx_step =0;
				f_rx_process =1;		//转入后台处理接收到的数据
			}
			break;
	}
}
void  process_rcv(uint16_t *p_buffer,uint8_t len)
{
	static uint8_t buff_cnt=0;
	p_buffer[buff_cnt++]=USART_ReceiveData(USART1);
	if(buff_cnt>=len)buff_cnt=0;
	while (USART_GetFlagStatus(USART1,USART_FLAG_TXE) == RESET);
	buff_cnt=0;
}
/* USART初始化 */
void USART1_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStruct;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIOA, ENABLE);  //使能GPIOA的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);//使能USART的时钟
	/* USART1的端口配置 */
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);//配置PA9成第二功能引脚	TX
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_1);//配置PA10成第二功能引脚  RX	

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	

	/* USART1的基本配置 */
	USART_InitStructure.USART_BaudRate = 115200;              //波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	USART_Init(USART1, &USART_InitStructure);		
//	USART_ITConfig(USART1,USART_IT_RXNE,ENABLE);           //使能接收中断
	USART_Cmd(USART1, ENABLE);                             //使能USART1
	
	/* USART1的NVIC中断配置 */
//	NVIC_InitStruct.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStruct.NVIC_IRQChannelPriority = 0x02;
//	NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//	NVIC_Init(&NVIC_InitStruct);
				
}




/**
  * @brief  Retargets the C library printf function to the USART.
  * @param  None
  * @retval None
  */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART */
  USART_SendData(USART1, (uint8_t) ch);

  /* Loop until the end of transmission */
  while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
  {}

  return ch;
}