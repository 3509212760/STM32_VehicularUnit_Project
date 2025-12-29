#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"
#include "FreeRTOS.h"
#include "adc.h"
#include "exti.h"
#include "lcd.h"
#include "gui.h"
#include "beep.h"
#include "us105.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 
#include "dma.h"
#include "stmflash.h"
#include "iap.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"


/*************************************************************************
项目名：基于FreeRTOS的车载电控单元(bootloader)
实现功能：请看用户手册
作者：不甘心的咸鱼--闲鱼
闲鱼号：tb43915564
修改日期：2025/3/20
请勿商用！
注：该工程代码与之前的Keil5_Project_updated_V1.2实现功能一样，
只不过该工程为FreeRTOS版本。
**************************************************************************/



/////////////////////////FreeRTOS任务设置////////////////////////////////////////////////////////////////////////////////////////
//START 任务
//设置任务优先级
#define START_TASK_PRIO      			1 /*开始任务的优先级设置为最低,与ucos只能相反，此处0为最低优先级，赋给IDLE任务*/
//设置任务堆栈大小
#define START_STK_SIZE  				64 /*单位是字，一个字是4个字节，半字是2个字节*/
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
static void start_task(void *pvParameters);	


//电机正向转动任务
//设置任务优先级
#define PWM_TASK_PRIO       			4
//设置任务堆栈大小
#define PWM_STK_SIZE  					64
//任务句柄
TaskHandle_t  PWMTask_Handler;
//任务函数
static void pwm_task(void *pvParameters);


//电机反向转动
#define Re_TASK_PRIO					5
#define	Re_STK_SIZE						64
TaskHandle_t  ReTask_Handler;
static void 	reverse_task(void *pvParameters);


//电机转速显示任务
#define Motoshow_task_prio   			7
#define Motoshow_task_size				64
TaskHandle_t	MotoshowTask_Handler;
static void Motoshow_task(void *pvParameters);


//us105测量距离任务
#define us105_task_prio       		  	6    
#define us105_task_size					128
TaskHandle_t	Us015Task_Handler;
static void us105_task(void *pvParameters);

//us_015启动任务
#define st_task_prio                 	2
#define st_task_size					32
TaskHandle_t	StTask_Handler;
static void st_task(void *pvParameters);

//MPU6050测量倾斜角度任务						
#define mpu6050_task_prio				3
#define	mpu6050_task_size				256
TaskHandle_t	Mpu6050Task_Handler;
static void mpu6050_task(void *pvParameters);

#if ifopen
//IAP任务
#define  iap_task_prio               	8
#define  iap_task_size            		512
TaskHandle_t	IapTask_Handler;
static void  iap_task(void *pvParameters);

#endif
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////定义全局变量///////////////////////////////////////////////////////////
/*思考一下为什么要用static关键字?*/
//定义需要用到的全局变量
static SemaphoreHandle_t 	sig1;
static SemaphoreHandle_t 	sig2;
static SemaphoreHandle_t 	sig3;
static QueueHandle_t 		sig4;	/*us015测量距离队列句柄*/
#if ifopen
static SemaphoreHandle_t 	sig5;
#endif
static led_d led1;	/*led结构体句柄--不甘心的咸鱼注*/
static led_d bep;
static led_d us;
static volatile u8 flag=8;
static volatile u8 mark=8;
static volatile u32 rate=0;
static volatile u32 buff;
static volatile u8  STA=0;
/********************************************************************************************/


/******************************定义APP程序缓冲区大小*************************************************************/
/**若使能为1，此时点击MAP文件会发现RAM大小为19.56KB，这是因为boot偏移了16K，再加上APP的3.56K，因此这里的RAM是boot加APP的合计***/
//定义接收的APP代码大小
#if ifopen		/*如要使用固件更新功能，请使能为1*/

	/*需要注意不能让bootloader程序的RAM区域与APP程序的RAM区域重叠，而笔者bootloader的RAM大概为12K，因此偏移了16K，为0x4000,剩下的4Kram给APP*/
	
	u8 receive_buff[buff_size]  __attribute__ ((at(0X20004000)));/*将数组分配到固定的内存地址（请根据需要修改）,否则会判断固件程序失败*/
	
#endif

/********************************************************************************************************************/



/*主函数，用于初始化硬件以及创建开始任务*/
 int main(void)
 {	
	/************************************************************************************************************/
	/*由于共计6个中断，设置中断分组为4，4位抢占优先级，0位亚优先级，因为Free不支持亚优先级处理。
	 由于Free中存在屏蔽优先级阈值的概念，该工程笔者设置为3，因此我们需要调用API函数的中断优先级不能超过3，
	 所以DMA中断优先级为4，输入捕获中断优先级为5
	 定时器2中断优先级为6，PB12/PC14外部中断优先级7，PB1外部中断优先级8，从优先级都设置为0--不甘心的咸鱼注*/
	/*************************************************************************************************************/
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//设置中断优先级分组为组4：4位抢占优先级，0位响应优先级 	
	delay_init();	    //延时函数初始化
	uart_init(115200);
	MPU_Init();
	mpu_dmp_init();
	LED_Init(&led1,GPIOC,GPIO_Pin_13);			 //初始化与LED连接的硬件接口
	Beep_Init(&bep,GPIOB,GPIO_Pin_15);			/*初始化beep*/
	us105Init(&us,GPIOA,GPIO_Pin_12);			/*初始化US-015*/
	TIM3_PWM_Init(99,14399);					/*初始化pwm*/
	EXTIX_Init();								/*初始化按键中断*/
	LCD_Init();									/*LCD初始化*/
#if ifopen
	MYDMA_Config(DMA1_Channel5,(u32)&USART1->DR,(u32)receive_buff,buff_size);//初始化DMA
#endif  
	xTaskCreate(start_task,"start_task",START_STK_SIZE,NULL,START_TASK_PRIO,&StartTask_Handler);/*创建起始任务*/
	vTaskStartScheduler(); 	/*开启多任务调度*/  	 
}
 

/*******************************************以下为任务部分，共计14个任务，在rtos启动后开始调度******************************************************************/	  

/*开始任务*/
static void start_task(void *pvParameters)
{
	/**为防止定时中断初始化以后进入中断函数，但此时信号量并没有创建
	,导致进入HardFault问题,因此在起始任务中才初始化定时器中断函数*/
	
	taskENTER_CRITICAL();			//进入临界区(无法被中断打断) 
	
	/**********创建队列*******************/
	sig4=xQueueCreate(1,sizeof(u16));
	/*************************************/
	
	/*****创建二值信号量--FreeRTOS********/
	sig1=xSemaphoreCreateBinary();
	sig2=xSemaphoreCreateBinary();
	sig3=xSemaphoreCreateBinary();
#if ifopen
	sig5=xSemaphoreCreateBinary();
#endif
	/**************************************/
	
	/********初始化定时器中断******************/
	TIM2_Int_Init(4999,14399);	
	/****************************************/
	
	/*****定时器输入捕获初始化*****************/
	TIM1_Cap_Init(0xFFFF,72-1);	/*72分频，测量精度为1us*/
	/*****************************************/
	
	/********创建任务**************************************************************************************/    				   
	xTaskCreate(pwm_task,"pwm_task",PWM_STK_SIZE,NULL,PWM_TASK_PRIO,&PWMTask_Handler);
	xTaskCreate(reverse_task,"reverse_task",Re_STK_SIZE,NULL,Re_TASK_PRIO,&ReTask_Handler);
	xTaskCreate(Motoshow_task,"motoshow_task",Motoshow_task_size,NULL,Motoshow_task_prio,&MotoshowTask_Handler);
	xTaskCreate(us105_task,"us015_task",us105_task_size,NULL,us105_task_prio,&Us015Task_Handler);
	xTaskCreate(st_task,"st_task",st_task_size,NULL,st_task_prio,&StTask_Handler);
	xTaskCreate(mpu6050_task,"mpu6050_task",mpu6050_task_size,NULL,mpu6050_task_prio,&Mpu6050Task_Handler);
#if ifopen
	xTaskCreate(iap_task,"iap_task",iap_task_size,NULL,iap_task_prio,&IapTask_Handler);
#endif
	vTaskDelete(StartTask_Handler); 	//删除开始任务
	/****************************************************************************************************/
	
	taskEXIT_CRITICAL();				//退出临界区(可以被中断打断)
}



/*电机正向转动任务*/
static void pwm_task(void *pvParameters)
{
	while(1)	/*笔者习惯用此来代表死循环，用户也可以选择用for(;;)*/
	{
		xSemaphoreTake(sig1,portMAX_DELAY);	/*等待信号量，portMAX_DELAY代表无限等*/
		TIM_SetCompare3(TIM3,flag);	/*电机正转*/
	}
}


/*电机反转任务*/
static void reverse_task(void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(sig2,portMAX_DELAY);	/*等待信号量，portMAX_DELAY代表无限等*/
		TIM_SetCompare3(TIM3,mark);	/*电机反转*/
	}
}


/*电机转速显示任务*/
static void Motoshow_task(void *pvParameters)
{
	while(1)
	{
		xSemaphoreTake(sig3,portMAX_DELAY);	/*等待信号量，portMAX_DELAY代表无限等*/
		buff=rate;
		/*显示车速*/
		Show_Str(0,80,BLUE,WHITE,"Speed:00r/min",16,0);
		LCD_ShowNum(0+6*8,80,60*buff/2,2,16);
		rate=0;/*车速值更新*/
	}
}


/*us105测量距离任务*/
static void us105_task(void *pvParameters)
{
	BaseType_t xResult;
	u16 PluseWidth,num;
	while(1)
	{
		/* 从队列中获取数据，等待时间为portMAX_DELAY，一直等待 */
        xResult = xQueueReceive(sig4, &PluseWidth, portMAX_DELAY);
        if(xResult == pdPASS){ 	/* 接收到消息 */
		/*printf("%d",PluseWidth);*/	/*调试用*/
		if(STA&0x80)
		{
			num=0.017*PluseWidth;/*计算距离值*/
			Show_Str(0,60,BLUE,WHITE,"juli:00cm",16,0);
			LCD_ShowNum(0+5*8,60,num,2,16);
			if(num<=5)
			{
				Beep_on(&bep);
				led_on(&led1);
				TIM_SetCompare3(TIM3,8);
			}	
			else
			{
				led_off(&led1);
				Beep_off(&bep);
			}
			STA=0;
		}
	  }
	}
}


/*us-015启动任务*/
static void st_task(void *pvParameters)
{
	while(1)
	{
		us105_Start(&us);/*us-015启动函数，给trigger引脚提供启动电平*/
		
		/*该延时函数是对vTaskDelay函数的封装，即为vTaskDelay,为相对延时*/
		delay_ms(1000);	//这个延时一定要加，因为这是us015的触发信号，但是触发信号的频率越高，输出的脉冲频率也越高，
		//导致进入中断的频率也会越高，会让中断过于频繁，让cpu资源耗尽.
	}
}
	

/*mpu6050测量倾斜角度任务*/
static void mpu6050_task(void *pvParameters)
{
	float pitch,roll,yaw; 
	int tmp;
	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)/*返回值为0，表示dmp处理数据正常*/
		{
			tmp = roll * 100;/*将角度转换为整数，方便用于计算整数部分/小数部分；*/
			/*如roll=12.30,此时tmp＝1230，方便计算tmp/100得到整数部分*/
			if (tmp < 0)
			{
					tmp = -tmp;
					Show_Str(0,40,BLUE,WHITE,"angle:-00.00d",16,0);
					LCD_ShowNum(0+7*8,40,tmp/100,2,16);
					LCD_ShowNum(0+10*8,40,tmp%100,2,16);
			}
			else
			{
					Show_Str(0,40,BLUE,WHITE,"angle:00.00d",16,0);
					LCD_ShowNum(0+6*8,40,tmp/100,2,16);//整数部分
					LCD_ShowNum(0+9*8,40,tmp%100,2,16);//小数部分
			}
			if(tmp>4500)/*由于tmp是*100后的数，那么roll>45度时，此时tmp值应该为4500*/
			{
					TIM_SetCompare3(TIM3,8);//表示的此时电机减速
					Show_Str(0,20,BLUE,WHITE,"Caution!",16,0);
			}
			else 	LCD_Fill(0,0,128,35,WHITE);//清屏
			delay_ms(100);//延时尽量短，保证读取数据的频率足够高，否则时间长了会看不到数据变化和FIFO溢出,最高延时不能超过300ms
		}
	}
}


/*正向加档任务*/
void EXTI1_IRQHandler(void)//外部中断1,按下加速
{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;/*释放二值信号量所需参数，初始化为pdFalse*/
		delay_xms(10);/*消抖,该延时不会引起任务调度*/
		if(KEY0==0)
			{
				flag-=1;
				if(flag<3)
				{
					flag=8;
				}
				/*显示车速挡位*/
				Show_Str(0,100,BLUE,WHITE,"Gear:",16,0);
				LCD_ShowNum(0+5*8,100,flag,2,16);
				/* 释放二值信号量，如果有任务正在等待该信号量，则唤醒任务 */
				xSemaphoreGiveFromISR(sig1, &xHigherPriorityTaskWoken);
			}
		EXTI_ClearITPendingBit(EXTI_Line1);/*清楚中断标志位*/
		/* 如果需要的话进行一次任务切换 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/*电机反转及电机转速任务*/
void EXTI15_10_IRQHandler(void)//外部中断PB12,PC14用于控制电机反转/电机转速计数
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;/*释放二值信号量所需参数，初始化为pdFalse*/
	delay_xms(10);	/*消抖,不会引发调度*/
	if(EXTI_GetITStatus(EXTI_Line12)!=RESET)
	{
	  if(KEY1==0)
	    {
				mark+=1;
				if(mark>13)
				{
					mark=8;
				}
				Show_Str(0,120,BLUE,WHITE,"Gear:",16,0);
				LCD_ShowNum(0+5*8,120,mark,2,16);
				xSemaphoreGiveFromISR(sig2, &xHigherPriorityTaskWoken);/*释放二值信号量*/
	    }
		EXTI_ClearITPendingBit(EXTI_Line12);
	}
	if(EXTI_GetITStatus(EXTI_Line14)!=RESET)
	{
		if(KEY2==0)
		{
			rate++;
		}
		EXTI_ClearITPendingBit(EXTI_Line14);
	}
		/* 如果需要的话进行一次任务切换 */
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/*获取1s的转速任务*/
void TIM2_IRQHandler(void)//定时器中断函数，定时1s计算此时的转速,需要注意的是中断函数里边不能写太复杂的东西，不然就会被卡住
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;/*释放二值信号量所需参数，初始化为pdFalse*/
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) //检查 TIM2 更新中断发生与否
	{
			TIM_ClearITPendingBit(TIM2, TIM_IT_Update ); //清除 TIM2 更新中断标志
			//led_on(&led1);/*调试用*/
			xSemaphoreGiveFromISR(sig3, &xHigherPriorityTaskWoken);/*释放二值信号量*/
	}
	/* 如果需要的话进行一次任务切换 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


/*输入捕获中断任务*/
//以下是us105测距模块，捕获传回来的高电平脉冲,捕获中断
void TIM1_CC_IRQHandler(void)
{ 
	static u16	PluseWidth;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
 	if((STA&0X80)==0)//还未成功捕获	
	{	  
		if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET)//捕获1发生捕获事件
			{	
				if(STA&0X40)		//捕获到一个下降沿 		
				{	  			
					STA|=0X80;		//标记成功捕获到一次高电平脉宽
					PluseWidth=TIM_GetCapture1(TIM1);
					TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Rising); //CC1P=0 设置为上升沿捕获
				}else  								//还未开始,第一次捕获上升沿
				{
					STA=0;			//清空
					PluseWidth=0;
					TIM_SetCounter(TIM1,0);
					STA|=0X40;		//因此程序实际上是等到上升沿中断的时候，在此处主动把这个比寄存器的值变成不是0的
					TIM_OC1PolarityConfig(TIM1,TIM_ICPolarity_Falling);		//CC1P=1 设置为下降沿捕获
				}		    
			}			     	    					   
 	}
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); //清除中断标志位
	/*向队列中发送数据 */
    xQueueSendFromISR(sig4, &PluseWidth, &xHigherPriorityTaskWoken);
	/*如果需要的话进行一次任务切换 */
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


#if ifopen
/*固件升级任务*/
static void iap_task(void *pdata)
{
	uint16_t receivedLength;
	while(1)
	{
		xSemaphoreTake(sig5,portMAX_DELAY);	/*等待信号量，portMAX_DELAY代表无限等*/
		//printf("执行到这里");//调试用
		receivedLength = GetReceivedDataLength();/*计算所接收的数据量，需要将该变量放在等待信号量函数后，以确保及时更新*/
		if(receive_buff[0])//看是否接收到了APP代码
		{
				printf("已接收到APP代码为%d\n",receivedLength);
				//printf("%s\n",receive_buff);//调试用
				/*拷贝、跳转*/
				if(((*(vu32*)(0X20004000+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
					{	 
						Show_Str(0,120,BLUE,WHITE,"Firmware updating!",16,0);/*打印信息在LCD屏幕上代表正在更新*/
						FLASH_ErasePage(FLASH_APP1_ADDR);/*先擦除APP区的flash，防止有数据残留--不甘心的咸鱼*/
						iap_write_appbin(FLASH_APP1_ADDR,receive_buff,buff_size);//更新FLASH代码   				
						printf("固件更新完成!\r\n");	
						printf("开始执行flash用户代码\r\n");
						if(((*(vu32*)(FLASH_APP1_ADDR+4))&0xFF000000)==0x08000000)//判断是否为0X08XXXXXX.
							{	 
									iap_load_app(FLASH_APP1_ADDR);//执行FLASH APP代码
							}
						else{
									printf("非FLASH应用程序,无法执行!\r\n");  
							}	
					}
					/*如果判断固件失败，先将接收APP数组情况，防止数据残留并重新开启DMA，用于下次更新--不甘心的咸鱼*/
				else{
						memset(receive_buff,0,buff_size);   //清空数组
						MYDMA_Enable(DMA1_Channel5);//开启一次DMA传输，用于下次更新
						printf("error!固件判断失败");
					}
		}
	}
}


/*接收APP程序任务*/
void DMA1_Channel5_IRQHandler(void) 
{
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;/*释放二值信号量所需参数，初始化为pdFalse*/
		// 检查DMA传输完成中断
		if (DMA_GetITStatus(DMA1_IT_TC5))
		{
        // 清除中断标志
        DMA_ClearITPendingBit(DMA1_IT_TC5);
		//printf("CNDTR: %d\n", DMA_GetCurrDataCounter(DMA1_Channel5)); // 调试用
		xSemaphoreGiveFromISR(sig5, &xHigherPriorityTaskWoken);/*释放二值信号量*/	
		}
		/* 如果需要的话进行一次任务切换 */
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif


