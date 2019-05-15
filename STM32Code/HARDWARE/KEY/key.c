#include "key.h"
/**************************************************************************
�������ܣ�������ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void KEY_Init(void)
{
	RCC->APB2ENR|=1<<2;               //ʹ��PORTAʱ��	   	 
	GPIOA->CRL&=0XFF0FFFFF;           
	GPIOA->CRL|=0X00800000;           
  GPIOA->ODR|=1<<5; //   ����	
} 
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�����˫���ȴ�ʱ��
����  ֵ������״̬ 0���޶��� 1������ 2��˫�� 
**************************************************************************/
u8 click_N_Double (u8 time)
{
		static	u8 flag_key,count_key,double_key;	
		static	u16 count_single,Forever_count;
	  if(KEY==0)  Forever_count++;   //������־λδ��1
     else        Forever_count=0;
		if(0==KEY&&0==flag_key)		flag_key=1;	
	  if(0==count_key)
		{
				if(flag_key==1) 
				{
					double_key++;
					count_key=1;	
				}
				if(double_key==2) 
				{
					double_key=0;
					count_single=0;
					return 2;//˫��ִ�е�ָ��
				}
		}
		if(1==KEY)			flag_key=0,count_key=0;
		
		if(1==double_key)
		{
			count_single++;
			if(count_single>time&&Forever_count<time)
			{
			double_key=0;
			count_single=0;	
			return 1;//����ִ�е�ָ��
			}
			if(Forever_count>time)
			{
			double_key=0;
			count_single=0;	
			}
		}	
		return 0;
}
/**************************************************************************
�������ܣ�����ɨ��
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������ 
**************************************************************************/
u8 click(void)
{
			static u8 flag_key=1;//�������ɿ���־
			if(flag_key&&KEY==0)
			{
			flag_key=0;
			return 1;	// ��������
			}
			else if(1==KEY)			flag_key=1;
			return 0;//�ް�������
}
/**************************************************************************
�������ܣ��������
��ڲ�������
����  ֵ������״̬ 0���޶��� 1������2s
**************************************************************************/
u8 Long_Press(void)
{
			static u16 Long_Press_count,Long_Press;
	    if(Long_Press==0&&KEY==0)  Long_Press_count++;   //������־λδ��1
      else                       Long_Press_count=0; 
		  if(Long_Press_count>200)		
			{
				Long_Press=1;	
				Long_Press_count=0;
				return 1;
			}				
			 if(Long_Press==1)     //������־λ��1
			{
				  Long_Press=0;
			}
			return 0;
}
/**************************************************************************
�������ܣ�ѡ�����е�ģʽ
��ڲ�������
����  ֵ����
**************************************************************************/
u8  select(void)
{
	  int Angle=260;
	  static u8 flag=1;
	  int count;
	  oled_show_once();  //OLED��ʾ
		Encoder_Temp=abs((short)TIM3->CNT);
    count=Encoder_Temp;
	  if(count<=Angle)								       Flag_Way=0;  //APPң��ģʽ
		else if(count>Angle&&count<=2*Angle)	 Flag_Way=1;  //PS2ң��ģʽ
		else if(count>2*Angle&&count<=3*Angle) Flag_Way=2;	//CCDѲ��ģʽ
		else if(count>3*Angle&&count<=4*Angle) Flag_Way=3;	//���Ѳ��ģʽ
	  else TIM3->CNT=0;
	  if(Flag_Next==1)OLED_Clear(),flag=0;  //���OLED��Ļ ������������
	  return flag;	  
}

