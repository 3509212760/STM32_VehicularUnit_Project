#include "us105.h"
#include "delay.h"
/***********************
项目名：US-015驱动
作者：不甘心的咸鱼--闲鱼
闲鱼号：tb43915564
修改日期：2025/3/20
请勿商用！
************************/   

/*function:初始化US-015的io口
prameter:结构体变量
prameter:端口（GPIOA-G）
prameter:管脚（0-16）
*/
void us105Init(led_d *io,gpioled port,u16 pin)
{
	if(port==GPIOA) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); 
	else if(port==GPIOB) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	else if(port==GPIOC) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	else if(port==GPIOD) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);
	else if(port==GPIOE) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE);
	else if(port==GPIOF) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOF, ENABLE);
	else if(port==GPIOG) RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOG, ENABLE);

	io->port=port;
	io->pin=pin;
	
	us105_config(io);
}

/*配置io口*/
void us105_config(led_d *io)
{
	GPIO_InitTypeDef  GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = io->pin;				 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 
	GPIO_Init(io->port, &GPIO_InitStructure);					

}


//us105的起始信号
void us105_Start(led_d *io)
{
	GPIO_SetBits(io->port,io->pin);
	delay_us(50);
	GPIO_ResetBits(io->port,io->pin);
}

