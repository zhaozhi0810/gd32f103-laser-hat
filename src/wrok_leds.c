

#include "includes.h"


//״̬��,���̵ƵĿ���
//��8��led��һ��4�� GPA0��ӦGREEN  GPA1��ӦORANGE
//�ߵ�ƽ����
/*
������������ѹ����3.6V���������
��������/�����ɣ��̵Ƴ���
��磺�̵�����
�豸���ϣ���Ƴ���
*/
//

//ָʾ�Ƶĳ�ʼ��
void Led_Show_Work_init(void)
{
		//ʱ��ʹ��
	rcu_periph_clock_enable(RCU_GPIOA);	
		
	gpio_init(GPIOA, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0 | GPIO_PIN_1);	
	//Ϩ��
	gpio_bit_reset(GPIOA, GPIO_PIN_8);
}

//��ɫָʾ�Ƶ���
void GreenLed_Show_Work_On(void)
{
	gpio_bit_reset(GPIOA, GPIO_PIN_0);
}

//��ɫָʾ��Ϩ��
void GreenLed_Show_Work_Off(void)
{
	gpio_bit_set(GPIOA, GPIO_PIN_0);
}


//��ɫָʾ�Ƶ���
void OrangeLed_Show_Work_On(void)
{
	gpio_bit_reset(GPIOA, GPIO_PIN_1);
}


//��ɫָʾ��Ϩ��
void OrangeLed_Show_Work_Off(void)
{
	gpio_bit_set(GPIOA, GPIO_PIN_1);
}


//���ֵ������ļ���йأ���������
#define FLASH_FRQ  25   //10ms����һ�Σ���500ms�ı�һ������250msһ��״̬

//��������˸
/*
	10 ms����һ��
	���տ���ʱ��Ҫ����������
	��ȫ�ֱ������ƵƵ���ɫ���Ϳ�������
*/
void Task_Led_Show_Work(void)
{
	static uint8_t n = 0;
//	static system_run_status_t status_rec = DEV_POWEROFF;  //��¼֮ǰ��״̬
	system_run_status_t status;

	status = get_system_run_status();
	
		//�ػ�״̬�²����Ƽ�����
	if(status == DEV_POWEROFF)
	{
		n = 0;
		GreenLed_Show_Work_Off();   //ȷ������ȫ���ر�
		OrangeLed_Show_Work_Off();  //�ػ�״̬�£���Ϩ��
		return;
	}
	else
	{
		//if(status_rec != status) //���ڵ�״̬���¼��״̬��һ����˵��״̬�����˱仯
//		{
//			if(status_rec != DEV_POWEROFF)  //�Ѿ���¼���ǿ�������
//			{
//				n = 0;   // ��Ҫ�Ǵ�0��ʼһ���µ�״̬������Ҳû��Ҫ���о�Ӱ�첻��
//			}
//			status_rec = status;
//		}
		
		if(status == DEV_CHARGE)  //�̵�����
		{
			OrangeLed_Show_Work_Off(); //��ƿ϶�Ҫ�ر�
			
			if(n<FLASH_FRQ)
			{
				GreenLed_Show_Work_On();
			}
			else if(n<(FLASH_FRQ*2))
			{
				GreenLed_Show_Work_Off();
			}
			else  //n��ֵ
				n = 0;
			
		}
		else if(status == DEV_CHARGE_OK)  //�����ɣ��̵Ƴ���
		{
			GreenLed_Show_Work_On();   //�̵Ƴ���
			OrangeLed_Show_Work_Off();
		}
		else if(status == DEV_RUN_NORMAL)  //�������У��̵Ƴ���
		{
			GreenLed_Show_Work_On();   //�̵Ƴ���
			OrangeLed_Show_Work_Off();
		}
		else if(status == DEV_VOL_LE36)  //��ѹ����3.6V���������
		{
			GreenLed_Show_Work_Off(); //�̵ƿ϶�Ҫ�ر�
			
			if(n<FLASH_FRQ)
			{
				OrangeLed_Show_Work_On();
			}
			else if(n<(FLASH_FRQ*2))
			{
				OrangeLed_Show_Work_Off();
			}
			else  //n��ֵ
				n = 0;
		}
		else if(status == DEV_BUG)  //��Ƴ���
		{
			GreenLed_Show_Work_Off();
			OrangeLed_Show_Work_On();   //��Ƴ���
		}
	}
	n++;
}
