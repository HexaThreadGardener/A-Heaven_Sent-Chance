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
 ALIENTEKս��STM32������ʵ��32
 MPU6050���ᴫ���� ʵ��
 ����֧�֣�www.openedv.com
 �Ա����̣�http://eboard.taobao.com 
 ��ע΢�Ź���ƽ̨΢�źţ�"����ԭ��"����ѻ�ȡSTM32���ϡ�
 ������������ӿƼ����޹�˾  
 ���ߣ�����ԭ�� @ALIENTEK
************************************************/

//����1����1���ַ� 
//c:Ҫ���͵��ַ�
void usart1_send_char(u8 c)
{   	
	while(USART_GetFlagStatus(USART1,USART_FLAG_TC)==RESET); //ѭ������,ֱ���������   
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
	u8 t=0,report=1;			//Ĭ�Ͽ����ϱ�
	u8 key;
	float pitch,roll,yaw; 		//ŷ����
	short aacx,aacy,aacz;		//���ٶȴ�����ԭʼ����
	short gyrox,gyroy,gyroz;	//������ԭʼ����
	short temp;					//�¶�	
	LED_GPIO_Config();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	 //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(500000);	 	//���ڳ�ʼ��Ϊ500000
	delay_init();	//��ʱ��ʼ��
	usmart_dev.init(72);		//��ʼ��USMART
	MPU_Init();					//��ʼ��MPU6050
	 
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
			temp=MPU_Get_Temperature();	//�õ��¶�ֵ
			printf("pitch=%f,roll=%f,yaw=%f,temperature=%d\n",pitch,roll,yaw,temp);
			MPU_Get_Accelerometer(&aacx,&aacy,&aacz);	//�õ����ٶȴ���������
			MPU_Get_Gyroscope(&gyrox,&gyroy,&gyroz);	//�õ�����������
			//printf("%d %d %d %d %d %d\n",aacx,aacy,aacz,gyrox,gyroy,gyroz);
			//if(report)mpu6050_send_data(aacx,aacy,aacz,gyrox,gyroy,gyroz);//���Զ���֡���ͼ��ٶȺ�������ԭʼ����
			//if(report)usart1_report_imu(aacx,aacy,aacz,gyrox,gyroy,gyroz,(int)(roll*100),(int)(pitch*100),(int)(yaw*10));	
		}
	} 	
}
 


