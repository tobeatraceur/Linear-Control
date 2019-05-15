#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/

u8 Flag_Way=0,Flag_Show=0,Flag_Stop=1,Flag_Next;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Encoder_A_EXTI,Flag_Direction;  
int Encoder_Temp;
float Velocity,Velocity_Set,Turn,Angle_Set;
int Motor_A,Motor_B,Servo,Target_A,Target_B;  //�������������           
int Voltage;                                //��ص�ѹ������صı���
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
u8 delay_50,delay_flag; 										//��ʱ����
float Velocity_KP=62,Velocity_KI=62;	       //�ٶȿ���PID����
int PS2_LX=128,PS2_LY=128,PS2_RX=128,PS2_RY=128,PS2_KEY;     //PS2ң�����
u16 ADV[128]={0};              
u8 Bluetooth_Velocity=30,APP_RX;                 //����ң���ٶȺ�APP���յ�����
u8 CCD_Zhongzhi,CCD_Yuzhi,PID_Send,Flash_Send;   //����CCD FLASH���
int Sensor_Left,Sensor_Middle,Sensor_Right,Sensor;//���Ѳ�����
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====ϵͳʱ������
	delay_init(72);                 //=====��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //=====������ʼ��
	OLED_Init();                    //=====OLED��ʼ��
	Encoder_Init_TIM2();            //=====�������ӿ�
	Encoder_Init_TIM3();            //=====��ʼ�������� 
	EXTI_Init();                    //=====�ⲿ�ж�
	while(select())	{	}	            //=====ѡ������ģʽ 
	delay_ms(300);                  //=====��ʱ����
	uart_init(72,128000);           //=====��ʼ������1
  Motor_PWM_Init(7199,0);  				//=====��ʼ��PWM 10KHZ������������� 
	Adc_Init();                     //=====��ص�ѹ����adc��ʼ��
	uart3_init(36,9600); 						//=====����3��ʼ�� ����
	 if(Flag_Way==1)
	{
		PS2_Init();											//=====PS2�ֱ���ʼ��
		PS2_SetInit();									//=====ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
	}
	else if(Flag_Way==2)ccd_Init();  //=====CCD��ʼ��
	else if(Flag_Way==3)ele_Init();  //=====��Ŵ�������ʼ��	
	Flash_Read();	                   //=====��ȡPID����
	Timer_Init(49,7199);             //=====��ʱ�жϳ�ʼ�� 
	while(1)
		{     
			   if(Flag_Way==1)
			   {
						PS2_LX=PS2_AnologData(PSS_LX);    //PS2���ݲɼ�    
						PS2_LY=PS2_AnologData(PSS_LY);
						PS2_RX=PS2_AnologData(PSS_RX);
						PS2_RY=PS2_AnologData(PSS_RY);
						PS2_KEY=PS2_DataKey();	
			   }
				 	if(Flash_Send==1)        //д��PID������Flash,��app���Ƹ�ָ��
					{
          	Flash_Write();	
						Flash_Send=0;	
					}	
					if(Flag_Show==0)         //ʹ��MiniBalance APP��OLED��ʾ��
					{
  						APP_Show();	
							oled_show();          //===��ʾ����
					}
					else                      //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
					{
				      DataScope();          //����MiniBalance��λ��
					}	
				  delay_flag=1;	
					delay_50=0;
	 	    	while(delay_flag);	     //ͨ����ʱ�ж�ʵ�ֵ�50ms��׼��ʱ				
	}
}
