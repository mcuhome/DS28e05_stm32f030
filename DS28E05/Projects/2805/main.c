/**
  ******************************************************************************
  * @file    RTC/RTC_Calendar/main.c 
  * @author  MCD Application Team
  * @version V1.4.0
  * @date    24-July-2014
  * @brief   Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "timer2.h"
#include "uart.h"
#include "DS28E05.h"



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Uncomment the corresponding line to select the RTC Clock source */
//#define RTC_CLOCK_SOURCE_LSE   /* LSE used as RTC source clock */
#define RTC_CLOCK_SOURCE_LSI  // LSI used as RTC source clock. The RTC Clock
                                // may varies due to LSI frequency dispersion


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/


/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/
unsigned char time_buff[50]={0};
unsigned char receive_buff[20]={0};
char write_buff[20]={0};

u8 f_Time2ms=0;

/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */
#define ADDRESS 0X20
int main(void)
{  
	/*!< At this stage the microcontroller clock setting is already configured, 
	this is done through SystemInit() function which is called from startup
	file (startup_stm32f0xx.s) before to branch to application main.
	To reconfigure the default setting of SystemInit() function, refer to
	system_stm32f0xx.c file
	*/ 
	unsigned char wr_temp=0;
	USART1_Init();
	delay_init();	 
	GPIO_DS28E05_Out_Mode_init();
	DS28E05_ReadRom(receive_buff);//读出内置的ID值
	for(int i=0;i<8;i++)
	{
		printf("ID[%d]= %d \n",i,receive_buff[i]);
		write_buff[i]=0x56;
	}

	delay_ms(100);	
	
	ds28e05WriteMemory(ADDRESS,write_buff,8);//写数据到Memory中
	while (1)
	{
		wr_temp=DS28E05_ReadMemory(ADDRESS,8,receive_buff);//读出Memory的值
		printf("wr_temp= %x \n",wr_temp);
		for(int i=0;i<8;i++)
		{
			printf("ID[%d]= 0x%x \n",i,receive_buff[i]);
			receive_buff[i]=0;
		}
		delay_ms(1000);
	}
}





#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
