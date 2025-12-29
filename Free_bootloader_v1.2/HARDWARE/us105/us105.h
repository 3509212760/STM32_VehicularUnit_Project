#ifndef __US105_H
#define __US105_H
#include "sys.h"
#include  "led.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//Mini STM32开发板
//看门狗 驱动代码		   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2010/5/30
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 正点原子 2009-2019
//All rights reserved
////////////////////////////////////////////////////////////////////////////////// 	  

void us105Init(led_d *io,gpioled port,u16 pin);
void us105_config(led_d *io);
void us105_Start(led_d *io);
#endif
