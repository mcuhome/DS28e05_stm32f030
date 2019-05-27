/******************** (C) COPYRIGHT  Դ�ع����� ********************************
 * �ļ���  ��Timer.c
 * ����    ����ʼ��ͨ�ö�ʱ��TIM3��ʵ��TIM3��ʱ���� 
 * ����    ��zhuoyingxingyu
 * �Ա�    ��Դ�ع�����http://vcc-gnd.taobao.com/
 * ��̳��ַ������԰��-Ƕ��ʽ������̳http://vcc-gnd.com/
 * �汾����: 2015-12-20
 * Ӳ������: ��
 * ���Է�ʽ��J-Link-OB
**********************************************************************************/

#include "timer2.h"
#include "led.h"
#include "stm32f0xx.h"
#include "main.h"
uint16_t Time3;

 /**
  * @file   TIM3_Config
  * @brief  ���ú����⣬��ʼ����ʱ��2������
  * @param  ��
  * @retval ��
  */
void TIM3_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;
    //ʹ��TIM3ʱ��
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//TIM3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���
	
    //��ʱ����ʱʱ��T���㹫ʽ��T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK
    TIM_TimeBaseStructure.TIM_Period = (10-1);//�Զ���װ��ֵ10--��ʱʱ��(10*4800/48M)s //��ʱ1����
    TIM_TimeBaseStructure.TIM_Prescaler =(480-1);//Ԥ��Ƶֵ��+1Ϊ��Ƶϵ�� 
    TIM_TimeBaseStructure.TIM_ClockDivision =0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	//ʹ��TIM3�ж�Դ
    TIM_Cmd(TIM3, ENABLE);  	//ʹ��TIMx����						 	
}

 /**
  * @file   TIM3_IRQHandler
  * @brief  ��ʱ��3�жϴ�����
  * @param  ��
  * @retval ��
  */



 /**
  * @file   TIM3_IRQHandler
  * @brief  ���������ж�
  * @param  ��
  * @retval ��
  */
void TIM3_delay(uint16_t s)
{
  Time3=s;
  while(Time3);
}
