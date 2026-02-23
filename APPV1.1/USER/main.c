#include "led.h"
#include "delay.h"
#include "sys.h"

 
 
/************************************************
 APP程序
 flash起始区为0x0800F000;
 固件升级功能为：实现小灯闪烁
************************************************/


 int main(void)
 {
	led_d led1;
	SCB->VTOR = FLASH_BASE | 0xF000;
	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级	
	//uart_init(115200);	 //串口初始化为115200
 	LED_Init(&led1,GPIOC,GPIO_Pin_13);				     //LED端口初始化
 	while(1)
	{
			led_on(&led1);
			delay_ms(500);
			led_off(&led1);
			delay_ms(500);	
	}	 
}


