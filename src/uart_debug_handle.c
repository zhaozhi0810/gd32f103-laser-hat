
/*
���ڴ�����cpu֮��Ĵ���ͨ��
	����ΪGD32�Ĵ���1��115200��8N1


���Դ��ڽ��յ����
0.�������ʱ���ַ���
1.��ѹ����
2.cpu�������¶ȣ�Һ�����¶ȣ���lcd����״̬������pwmֵ��ioģʽ��ֻ��0��100��
3.lcd������ֵ�����ļӵ�����״̬��pd_n��״̬
4.4·diֵ��4·��ͨ·��Ϣ
5.Ӳ�����Ź�״̬�����ź�Դ�����ޣ�
6.cpu����״̬���������ػ�������������pmon������ϵͳ�ȣ�

����ʶ�������Ҳ�Ǵ�ӡ��ʾ�ͱ���ʱ���ַ���
*/



#include "includes.h"



//#define RECV_BUF_LEN 64
#if 0
static Queue_UART_STRUCT g_Queue_Debug_Recv;   //����Debug���ݶ��У����ڽ����ж�

frame_buf_t g_com_debug_buf={{0},FRAME_LENGHT};    //���ݴ�����
#endif



//�������ü�������仯
static void laser_area_control(uint8_t area,uint8_t add)
{
	if(area < '7' && area >= '0')
	{
		if(add)
			laser_add_a_area(area-'0');
		else
			laser_sub_a_area(area-'0');
	}
	printf("laser_area_control  area = %d  add= %d\r\n",area-'0',add);	
}



//�������ü�������仯
static void laser_pwm_control(uint8_t pwm)
{
	uint8_t degree;
	if(pwm <= '9' && pwm >= '0')
	{
		degree = ((pwm - '0')+ 1 )*10;
		pwm_all_change(degree);
	}
	printf("laser_pwm_control  degree = %d \r\n",degree);	
}


//�����������ֱ仯
static void wt588d_control(uint8_t buf)
{
	if(buf == '1') //��һ��
	{
		wt588d_playNextSound();
	}
	else if(buf == '2') //��һ��
	{
		wt588d_playLastSound();
	}
	else if(buf == '3') //������
	{
		wt588d_setVolume_Acc();
	}
	else if(buf == '4') //������
	{
		wt588d_setVolume_Dec();
	}
	printf("wt588d_control  buf = %d\r\n",buf);	
}



const char* sys_run_status[]= {"DEV_BUG" ,     //����
	"DEV_VOL_LE30 ",    //��ѹ����3.0	
	"DEV_POWEROFF",      //�ػ�
	"DEV_VOL_LE36 ",   //��ѹ����3.6	
	"DEV_RUN_NORMAL",  //��������
	"DEV_CHARGE",   //���
	"DEV_CHARGE_OK" ,  //������
	"DEV_EXTERN_POWER"   //�ⲿ���磬��ʱ��Ƭ���ǲ���ʡ���};
};


//�����������������Դ��ڽ��յ��ļ򵥵ĵ�������
static void Com_Debug_Message_Handle1(uint8_t buf)
{
	static uint8_t cmd = 0;   //�������ָ����ָ��
	uint8_t i;
	
	if(cmd == 0)
	{
		switch(buf)
		{
			default:   //cmd��ӡ��ʱ�򣬿��ܳ����˿���ʾ�ַ�������
				printf("ERROR: Command Unknow cmd = 0x%x!!!\r\n",buf);   //����ʶ�������
			case '0':
				printf("%s\r\n",g_build_time_str);  //��ӡ�����ʱ��
			break;
			case '1':
				printf("laser area = 0x%x \r\n",get_laser_area_val());
				//��ӡ����ֵ
				for(i=0;i<7;i++)  //������ӡ7��ֵ
					printf("laser pwm[%d] = %d \r\n",i,g_pwm[i]);
				break;
			case '2':
				//��ӡ�¶�ֵ
				printf("temp = %0.2f,humi = %0.2f\r\n",g_temperature,g_humidity);
				break;
			case '3':
				printf("usb charge status %d��0:nc,1:charging,2:full��\r\n",is_power_charge());  //usb�ӵ�״̬
				printf("system run status %s \r\n",sys_run_status[get_system_run_status()-1]);  //ϵͳ���е�״̬
	//			printf("lcd light pwm = %d\r\n",g_lcd_pwm);   //lcd������pwmֵ
				break;
			case '4':
	//			t = Get_Di_4Ttl_Status();   //4·����������DI PB12-PB15
	//			printf("4Di Di1 %s,Di2 %s,Di3 %s,Di4 %s \r\n",t&1?"on":"off",t&2?"on":"off",t&4?"on":"off",t&8?"on":"off");
	//			t = Get_Optica_Switch_Status();  //4·�⿪��״̬
	//			printf("4 op switch D2_STATE2 %s ,D2_STATE1 %s,D1_STATE1 %s,D1_STATE2 %s \r\n",
	//								t&1?"on":"off",t&2?"on":"off",t&4?"on":"off",t&8?"on":"off");
				break;
			case '5':
				printf("Watch Dog Status = %s\r\n","off");   //��ʱû�п���
				break;
			case '6':
	//			printf("Cpu Run Status = %s\r\n",g_Cpu_Run_Status_str[g_cpu_run_status-1]);
				break;
			case 'a':
			case 'A':   //���ü�����������
				cmd = 1;	
				break;
			case 'b':
			case 'B':   //���ü����������
				cmd = 2;
			case 'c':
			case 'C':   //���ü�������pwm�仯
				cmd = 4;	
				break;
			case 'd':
			case 'D':   //���ü�������pwm����
				cmd = 5;
			case 'm':
			case 'M':   //wt588d ������������һ����һ������������
				cmd = 3;
				break;
		}
	}
	else if(cmd == 1)  //���ü�����������
	{
		laser_area_control(buf,1);
		cmd = 0;   //�������ģʽ
	}
	else if(cmd == 2)  //���ü����������
	{
		laser_area_control(buf,0);
		cmd = 0;   //�������ģʽ
	}
	else if(cmd == 3)  //�������� ��һ�ף���һ�ף������������ּ�
	{
		wt588d_control(buf);
		cmd = 0;   //�������ģʽ
	//	printf("exit wt588d_control mode\r\n");		
	}
	else if(cmd == 4)  //���ü�����������
	{
		laser_pwm_control(buf);
		cmd = 0;   //�������ģʽ
	}
	else  //��ֹ�������ɿ�������
		cmd = 0;
//	else if(cmd == 5)   //���ü����������
//	{
//		
//		cmd = 0;   //�������ģʽ
//	}
//	else if(cmd == 6)
//	{
//		
//		cmd = 0;   //�������ģʽ
//	}
}


/*
	�������ݽ����жϣ�
		ǰ�᣺ÿһ֡����7���ֽڡ�
		�����б���֡ͷ���к�������ݺ�У��ͣ���7���ֽڣ�
*/
void Com_Debug_Rne_Int_Handle(void)
{
	uint8_t dat;

	dat = (uint8_t)usart_data_receive(EVAL_COM0);//(USART3);  
	Com_Debug_Message_Handle1(dat);   //ֱ�Ӵ���
//	QueueUARTDataInsert(&g_Queue_Debug_Recv,dat);   //���յ����ݴ�������С�
}


/*
	�����յ������Ĵ������ڵĿ����жϴ���������
		ǰ�᣺ �յ����������ݰ���У�����ȷ��

	��Ƭ���ܹ��յ������
	// 1.������ƵԴ,û�иù���
	4.����lcd��pwm�����ȣ�
	5.�ػ����������

*/

#if 0

static void Com_Debug_Message_Handle(uint8_t* buf)
{		
	com_frame_t* pdata = (com_frame_t*)(buf+1);    //+1������֡ͷ��ʹ�ýṹ���ʼ��
	int8_t t;
	
	switch(pdata->data_type)
    {
        case eMCU_CMD_TYPE:    //cpu���͸���Ƭ���Ķ���cmd����
            t = pdata->data.cmd.cmd;
            switch(t)
            {
				case eMCU_CPUGETINFO_CMD:   //��ȡ�豸��Ϣ������
				//	AnswerCpu_GetInfo(pdata->data.cmd.param1<<8 | pdata->data.cmd.param2); //ʹ�ú�������������������
					break;
				case eMCU_CPUSET_CMD:    //������Ļ����
					if(pdata->data.cmd.param1 == eMCU_LCD_SETPWM_CMD)
					{
						t = pdata->data.cmd.param2;   //���ֵ�����ɸ���������������������������
						t = g_lcd_pwm + t;   //����ó��µĽ��
						Lcd_pwm_out(t);     //��������pwm��ֵ
				//		AnswerCpu_Status(eUART_SUCCESS);   //Ӧ��ɹ�
					}
					else if(pdata->data.cmd.param1 == eMCU_SWITCH_DVI_SRC_CMD) //�л���ƵԴ
					{
						t = pdata->data.cmd.param2;  //0 Ϊ���أ�1Ϊ�ⲿ
//						if(t)
//							dvi_switch_set(DVI_OTHER);   //���ú���ϱ���cpu
//						else
//							dvi_switch_set(DVI_LOONGSON);   //������Ƶ
				//		AnswerCpu_Status(eUART_SUCCESS);   //Ӧ��ɹ�
					}
					else	
				//		AnswerCpu_Status(eUART_ERR_PARAM);  //Ӧ���������				
				break;
                default:
					DBG_PRINTF("ERROR: %s\n","eUART_ERR_PARAM");
				//	AnswerCpu_Status(eUART_ERR_PARAM);  //Ӧ���������
                break;
            }

        break;
        default:
			DBG_PRINTF("ERROR: %s\n","eUART_ERR_CMD_UNKNOW");
		//	AnswerCpu_Status(eUART_ERR_CMD_UNKNOW);  //Ӧ������δ֪ 
        break;
    }	
}
#endif



/*
	���ڿ����жϵĴ���,���Դ��ڲ��ٿ��������ж�

	1.�жϽ��յ����ֽ�����>=7 ��ʾ����
	2.�����ͼ�����������7���ֽڣ�����У��ͣ�
	3.У�����ȷ����������
*/
void Com_Debug_Idle_Int_Handle(void)
{
//	Com_Frame_Handle(&g_com_debug_buf, &g_Queue_Debug_Recv,Com_Debug_Message_Handle);
}


