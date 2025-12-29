#include "beep.h"
#include "usart.h"	 

/***********************
项目名：BEEP驱动
作者：不甘心的咸鱼--闲鱼
闲鱼号：tb43915564
修改日期：2025/3/20
请勿商用！
************************/   
 
 
 /*function:初始化BEEP的io口
prameter:结构体变量
prameter:端口（GPIOA-G）
prameter:管脚（0-16）
*/
void Beep_Init(led_d *yinjiao,gpioled port,u16 pin)
{
	if(port==GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	else if(port==GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if(port==GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if(port==GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	else if(port==GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	else if(port==GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	else if(port==GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

	
	yinjiao->port=port;
	yinjiao->pin=pin;
	
	Beep_config(yinjiao);
}

/*配置io口*/
void Beep_config(led_d *yinjiao)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = yinjiao->pin;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(yinjiao->port, &GPIO_InitStructure);					
	Beep_off(yinjiao);
}

//关闭蜂鸣器
void Beep_off(led_d *yinjiao)
{
	GPIO_ResetBits(yinjiao->port,yinjiao->pin);
}


//打开蜂鸣器
void Beep_on(led_d *yinjiao)
{
	GPIO_SetBits(yinjiao->port,yinjiao->pin);
}
