
/*
用于处理与cpu之间的串口通信
	串口为GD32的串口1，115200，8N1


调试串口接收的命令：
0.软件编译时间字符串
1.电压电流
2.cpu和主板温度，液晶屏温度，及lcd加热状态，风扇pwm值（io模式下只有0和100）
3.lcd的亮度值，屏的加电引脚状态，pd_n的状态
4.4路di值，4路光通路信息
5.硬件看门狗状态，（信号源，暂无）
6.cpu运行状态。（开机关机，重启，进入pmon，进入系统等）

不能识别的命令也是打印提示和编译时间字符串
*/



#include "includes.h"



//#define RECV_BUF_LEN 64
#if 0
static Queue_UART_STRUCT g_Queue_Debug_Recv;   //接收Debug数据队列，用于接收中断

frame_buf_t g_com_debug_buf={{0},FRAME_LENGHT};    //数据处理缓存
#endif



//串口设置激光区域变化
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



//串口设置激光区域变化
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


//串口设置音乐变化
static void wt588d_control(uint8_t buf)
{
	if(buf == '1') //下一首
	{
		wt588d_playNextSound();
	}
	else if(buf == '2') //上一首
	{
		wt588d_playLastSound();
	}
	else if(buf == '3') //音量增
	{
		wt588d_setVolume_Acc();
	}
	else if(buf == '4') //音量减
	{
		wt588d_setVolume_Dec();
	}
	printf("wt588d_control  buf = %d\r\n",buf);	
}



const char* sys_run_status[]= {"DEV_BUG" ,     //故障
	"DEV_VOL_LE30 ",    //电压低于3.0	
	"DEV_POWEROFF",      //关机
	"DEV_VOL_LE36 ",   //电压低于3.6	
	"DEV_RUN_NORMAL",  //正常运行
	"DEV_CHARGE",   //充电
	"DEV_CHARGE_OK" ,  //充电完成
	"DEV_EXTERN_POWER"   //外部供电，这时单片机是不会省电的};
};


//这个函数用来处理调试串口接收到的简单的调试命令
static void Com_Debug_Message_Handle1(uint8_t buf)
{
	static uint8_t cmd = 0;   //用于区分更多的指令
	uint8_t i;
	
	if(cmd == 0)
	{
		switch(buf)
		{
			default:   //cmd打印的时候，可能超出了可显示字符的区间
				printf("ERROR: Command Unknow cmd = 0x%x!!!\r\n",buf);   //不能识别的命令
			case '0':
				printf("%s\r\n",g_build_time_str);  //打印编译的时间
			break;
			case '1':
				printf("laser area = 0x%x \r\n",get_laser_area_val());
				//打印电流值
				for(i=0;i<7;i++)  //连续打印7个值
					printf("laser pwm[%d] = %d \r\n",i,g_pwm[i]);
				break;
			case '2':
				//打印温度值
				printf("temp = %0.2f,humi = %0.2f\r\n",g_temperature,g_humidity);
				break;
			case '3':
				printf("usb charge status %d（0:nc,1:charging,2:full）\r\n",is_power_charge());  //usb加电状态
				printf("system run status %s \r\n",sys_run_status[get_system_run_status()-1]);  //系统运行的状态
	//			printf("lcd light pwm = %d\r\n",g_lcd_pwm);   //lcd的亮度pwm值
				break;
			case '4':
	//			t = Get_Di_4Ttl_Status();   //4路开关量输入DI PB12-PB15
	//			printf("4Di Di1 %s,Di2 %s,Di3 %s,Di4 %s \r\n",t&1?"on":"off",t&2?"on":"off",t&4?"on":"off",t&8?"on":"off");
	//			t = Get_Optica_Switch_Status();  //4路光开关状态
	//			printf("4 op switch D2_STATE2 %s ,D2_STATE1 %s,D1_STATE1 %s,D1_STATE2 %s \r\n",
	//								t&1?"on":"off",t&2?"on":"off",t&4?"on":"off",t&8?"on":"off");
				break;
			case '5':
				printf("Watch Dog Status = %s\r\n","off");   //暂时没有开启
				break;
			case '6':
	//			printf("Cpu Run Status = %s\r\n",g_Cpu_Run_Status_str[g_cpu_run_status-1]);
				break;
			case 'a':
			case 'A':   //设置激光区域增加
				cmd = 1;	
				break;
			case 'b':
			case 'B':   //设置激光区域减少
				cmd = 2;
			case 'c':
			case 'C':   //设置激光区域pwm变化
				cmd = 4;	
				break;
			case 'd':
			case 'D':   //设置激光区域pwm减少
				cmd = 5;
			case 'm':
			case 'M':   //wt588d 调整，音乐上一曲下一曲，音量增减
				cmd = 3;
				break;
		}
	}
	else if(cmd == 1)  //设置激光区域增加
	{
		laser_area_control(buf,1);
		cmd = 0;   //清除命令模式
	}
	else if(cmd == 2)  //设置激光区域减少
	{
		laser_area_control(buf,0);
		cmd = 0;   //清除命令模式
	}
	else if(cmd == 3)  //设置音乐 上一首，下一首，音乐增，音乐减
	{
		wt588d_control(buf);
		cmd = 0;   //清除命令模式
	//	printf("exit wt588d_control mode\r\n");		
	}
	else if(cmd == 4)  //设置激光区域增加
	{
		laser_pwm_control(buf);
		cmd = 0;   //清除命令模式
	}
	else  //防止其他不可靠的问题
		cmd = 0;
//	else if(cmd == 5)   //设置激光区域减少
//	{
//		
//		cmd = 0;   //清除命令模式
//	}
//	else if(cmd == 6)
//	{
//		
//		cmd = 0;   //清除命令模式
//	}
}


/*
	串口数据接收中断：
		前提：每一帧都是7个字节。
		队列中保存帧头，有后面的数据和校验和（共7个字节）
*/
void Com_Debug_Rne_Int_Handle(void)
{
	uint8_t dat;

	dat = (uint8_t)usart_data_receive(EVAL_COM0);//(USART3);  
	Com_Debug_Message_Handle1(dat);   //直接处理
//	QueueUARTDataInsert(&g_Queue_Debug_Recv,dat);   //接收的数据存入队列中。
}


/*
	串口收到命令后的处理。串口的空闲中断处理函数调用
		前提： 收到完整的数据包，校验和正确。

	单片机能够收到的命令：
	// 1.设置视频源,没有该功能
	4.设置lcd的pwm（亮度）
	5.关机或重启命令。

*/

#if 0

static void Com_Debug_Message_Handle(uint8_t* buf)
{		
	com_frame_t* pdata = (com_frame_t*)(buf+1);    //+1是跳过帧头，使用结构体初始化
	int8_t t;
	
	switch(pdata->data_type)
    {
        case eMCU_CMD_TYPE:    //cpu发送给单片机的都是cmd！！
            t = pdata->data.cmd.cmd;
            switch(t)
            {
				case eMCU_CPUGETINFO_CMD:   //获取设备信息的命令
				//	AnswerCpu_GetInfo(pdata->data.cmd.param1<<8 | pdata->data.cmd.param2); //使用函数解析，并返回数据
					break;
				case eMCU_CPUSET_CMD:    //设置屏幕亮度
					if(pdata->data.cmd.param1 == eMCU_LCD_SETPWM_CMD)
					{
						t = pdata->data.cmd.param2;   //这个值可正可负，根据它的正负来调亮或者灭
						t = g_lcd_pwm + t;   //计算得出新的结果
						Lcd_pwm_out(t);     //重新设置pwm的值
				//		AnswerCpu_Status(eUART_SUCCESS);   //应答成功
					}
					else if(pdata->data.cmd.param1 == eMCU_SWITCH_DVI_SRC_CMD) //切换视频源
					{
						t = pdata->data.cmd.param2;  //0 为本地，1为外部
//						if(t)
//							dvi_switch_set(DVI_OTHER);   //设置后会上报给cpu
//						else
//							dvi_switch_set(DVI_LOONGSON);   //本地视频
				//		AnswerCpu_Status(eUART_SUCCESS);   //应答成功
					}
					else	
				//		AnswerCpu_Status(eUART_ERR_PARAM);  //应答参数错误				
				break;
                default:
					DBG_PRINTF("ERROR: %s\n","eUART_ERR_PARAM");
				//	AnswerCpu_Status(eUART_ERR_PARAM);  //应答参数错误
                break;
            }

        break;
        default:
			DBG_PRINTF("ERROR: %s\n","eUART_ERR_CMD_UNKNOW");
		//	AnswerCpu_Status(eUART_ERR_CMD_UNKNOW);  //应答命令未知 
        break;
    }	
}
#endif



/*
	串口空闲中断的处理,调试串口不再开启空闲中断

	1.判断接收到的字节数，>=7 表示正常
	2.正常就继续处理，读出7个字节，计算校验和，
	3.校验和正确，则处理命令
*/
void Com_Debug_Idle_Int_Handle(void)
{
//	Com_Frame_Handle(&g_com_debug_buf, &g_Queue_Debug_Recv,Com_Debug_Message_Handle);
}


