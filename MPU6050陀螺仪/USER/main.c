#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "lcd.h"
#include "usart.h"
#include "mpu6050.h"
#include "usmart.h"   
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h" 

 
/************************************************
 ALIENTEK战舰STM32开发板实验32
 MPU6050六轴传感器 实验
 技术支持：www.openedv.com
 淘宝店铺：http://eboard.taobao.com 
 关注微信公众平台微信号："正点原子"，免费获取STM32资料。
 广州市星翼电子科技有限公司  
 作者：正点原子 @ALIENTEK
************************************************/

//串口1发送1个字符 
//c:要发送的字符
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //循环发送,直到发送完毕   
	USART_SendData(USART1,c);  
} 
 void turn(float degree)//degree>0 clockwise <0 anticlockwise;
 {
	 float pitch,roll,InitDegree,CurrentDegree;
	 int targetDegree,temp;
	 while(1)
	 {
		if(mpu_dmp_get_data(&pitch,&roll,&InitDegree)==0)
			break;
	 }
	 //InitDegree=0;
	 if(InitDegree<0) InitDegree+=360;
	 temp=(int)(InitDegree+degree);
	 while(temp<0)
		 temp+=360;
	 targetDegree=temp %360;
	 while(targetDegree>180)
		 targetDegree-=360;
	 if (InitDegree>180) InitDegree-=360;
	 printf("targetDegree=%d\n",targetDegree);
	 while(1)
	 {
		 mpu_dmp_get_data(&pitch,&roll,&CurrentDegree);
		 if (targetDegree>0)
		 {
			 if ((CurrentDegree-targetDegree)>=-180 && (CurrentDegree-targetDegree)<=0)
			 {
				 //Motor(1,100),Motor(2,-100);Motor clockwise turn;
				 printf("clockwise turn!\n");
			 }
			 else
			 {
				 //Motor(1,100),Motor(2,-100);Motor clockwise turn;
				 printf("anticlockwise turn\n");
			 }
		 }
		 if (targetDegree<=0)
		 {
			 if ((CurrentDegree-targetDegree)>=0 && (CurrentDegree-targetDegree)<=180)
			 {
				 //Motor(1,100),Motor(2,-100);Motor clockwise turn;
				 printf("anticlockwise turn!\n");
			 }
			 else
			 {
				 //Motor(1,-100),Motor(2,100);Motor clockwise turn;
				 printf("clockwise turn\n");
			 }
		 }
		 if (abs((int)(targetDegree-CurrentDegree))<15)
		 {
			GPIO_SetBits(GPIOD,GPIO_Pin_2);
			break;
		 }
		 printf("init=%f current=%f target=%d\n",InitDegree,CurrentDegree,targetDegree);
		 delay_ms(200);
	 }
 }
 int main(void)
 {	 
	u8 t=0,report=1;			//默认开启上报
	u8 key;
	float pitch,roll,yaw; 		//欧拉角
	short aacx,aacy,aacz;		//加速度传感器原始数据
	short gyrox,gyroy,gyroz;	//陀螺仪原始数据
	short temp;					//温度	
	LED_GPIO_Config();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(500000);	 	//串口初始化为500000
	delay_init();	//延时初始化
	usmart_dev.init(72);		//初始化USMART
	MPU_Init();					//初始化MPU6050
	 
//	while(1)
//	{
//			GPIO_ResetBits(GPIOD,GPIO_Pin_2);  //LED0????GPIOB.5??,?  ??LED0=0;
//			delay_ms(300);  		   //??300ms
//			GPIO_SetBits(GPIOD,GPIO_Pin_2);	   //LED0????GPIOB.5??,?  ??LED0=1;
//			delay_ms(300);                     //??300ms
//	}
	GPIO_ResetBits(GPIOD,GPIO_Pin_2);
	while(key=mpu_dmp_init())
 	{
		printf("hello world key=%d\n",key);
		delay_ms(200);
	}
	while(1)
	{
		turn(90);
		delay_ms(5000);
	}

 	while(1)
	{
		if(mpu_dmp_get_data(&pitch,&roll,&yaw)==0)
		{ 
			delay_ms(500);
			temp=MPU_Get_Temperature();	//得到温度值
			printf("pitch=%f,roll=%f,yaw=%f,temperature=%d\n",pitch,roll,yaw,temp);
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//得到加速度传感器数据
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//得到陀螺仪数据
			//printf("%d %d %d %d %d %d\n",aacx,aacy,aacz,gyrox,gyroy,gyroz);
			//if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//用自定义帧发送加速度和陀螺仪原始数据
			//if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	
		}
	} 	
}
 


