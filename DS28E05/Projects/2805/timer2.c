/******************** (C) COPYRIGHT  源地工作室 ********************************
 * 文件名  ：Timer.c
 * 描述    ：初始化通用定时器TIM3，实现TIM3定时功能 
 * 作者    ：zhuoyingxingyu
 * 淘宝    ：源地工作室http://vcc-gnd.taobao.com/
 * 论坛地址：极客园地-嵌入式开发论坛http://vcc-gnd.com/
 * 版本更新: 2015-12-20
 * 硬件连接: 无
 * 调试方式：J-Link-OB
**********************************************************************************/

#include "timer2.h"
#include "led.h"
#include "stm32f0xx.h"
#include "main.h"
uint16_t Time3;

 /**
  * @file   TIM3_Config
  * @brief  调用函数库，初始化定时器2的配置
  * @param  无
  * @retval 无
  */
void TIM3_Config(void)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef         NVIC_InitStructure;
    //使能TIM3时钟
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); 
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);
    NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器
	
    //定时器定时时间T计算公式：T=(TIM_Period+1)*(TIM_Prescaler+1)/TIMxCLK
    TIM_TimeBaseStructure.TIM_Period = (10-1);//自动重装载值10--定时时间(10*4800/48M)s //定时1毫秒
    TIM_TimeBaseStructure.TIM_Prescaler =(480-1);//预分频值，+1为分频系数 
    TIM_TimeBaseStructure.TIM_ClockDivision =0; 
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;	//TIM向上计数模式
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
    TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE);	//使能TIM3中断源
    TIM_Cmd(TIM3, ENABLE);  	//使能TIMx外设						 	
}

 /**
  * @file   TIM3_IRQHandler
  * @brief  定时器3中断处理函数
  * @param  无
  * @retval 无
  */



 /**
  * @file   TIM3_IRQHandler
  * @brief  产生毫秒中断
  * @param  无
  * @retval 无
  */
void TIM3_delay(uint16_t s)
{
  Time3=s;
  while(Time3);
}
