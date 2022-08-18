
#include "includes.h"
#include <string.h>

/*
	串口蓝牙  PA2，3
	
	蓝牙连接之后是数据透传，AT指令无效
	
CC41的AT指令是这样的：
指令不分大小写；
指令最后要跟上换行也就是“\r\n”（忘记加上会没有反应）；
设置参数直接跟在指令后面，如at+role1，就是将模块设置为主机。

HM-10的AT指令是这样的：
指令全都要大写；
指令最后不要跟上换行“\r\n”（有换行的话模块会没有反应）；
查询类语句后面要加上“?”，如AT+ROLE?是查询当前模块是主机(Master)还是从机(Slave)；
设置参数直接跟在指令后面，如AT+ROLE1，就是将模块设置为主机。

HC-08的AT指令是这样的：
指令全都要大写；
指令最后不要跟上换行“\r\n”（有换行的话模块会没有反应）；
查询类语句后面要加上“=?”，如AT+ROLE=?是查询当前模块是主机(Master)还是从机(Slave)；
设置参数在指令后面要加上“=”，如AT+ROLE=M，就是将模块设置为主机。

HC-05的AT指令则是基本跟HM-10一样，问题是要求跟上换行“\r\n”。


在HM-10的AT+DISC表示扫描，而在HC-05则是表示断开连接。
HM-10就是在连接之后再发“AT”指令，连接就会自动断开。
HC-05则是用“AT+DISC”。
估计还要以手册为准！！！



HM-10模块的3种工作模式，HM-10模块功能非常强大，它的几个管脚是PIO，可以用于输出或采集。所以为了设置它的PIO的工作状态，有一个指令是AT+MODE。
AT+MODE0: 透传模式（这是默认值，还记得上一部分中刷完模块要发AT+MODE1吗？因为默认值下它对AT指令没有反应。）
1.就是不会接收AT指令，直接将收到的内容转发。

AT+MODE1: PIO 采集+远控+透传
1.PIO2~3这两个管脚为输出；
2.PIO4~11这8个管脚为输入；（采集数据，具体采集什么数据，请自行查看HM-10的说明书）
3.接收AT指令；
4.非“AT”开头的字符串直接转发。

AT+MODE2：透传+远控模式
1.PIO2~11这两个管脚为输出；
2.接收AT指令；
3.非“AT”开头的字符串直接转发。

*/

const uint8_t* bt_cmd_end = "\r\n";

const uint8_t* bt_cmd[] = {   "AT"        // 1.测试指令
					,"AT+VERSION"      //2.获取软件版本号
					,"AT+LADDR"      //3.获取模块蓝牙地址
					,"AT+NAME"      //4.设置/查询设备名称
					,"AT+PIN"      //5.设置/查询-配对码
					,"AT+TYPE"      //6.设置/查询-模块鉴权工作类型
					,"AT+BAUD"      //7.设置/查询-串口波特率
					,"AT+ADVI"      //8.设置/查询-广播间隔
					,"AT+POWE"      //9.设置模块发射功率
					,"AT+UUID"      //10.设置/查询-Service UUID
					,"AT+CHAR"     //11.设置/查询-Characteristic
					,"AT+RESET"    //12.软件重启（500ms 后重启）
					,"AT+ROLE"     //13.设置/查询-主/从模式
					,"AT+SLEEP"    //14.进入低功耗模式（进入后可被搜索）
					,"AT+INQ"      //15.搜索蓝牙设备（主模式指令）
					,"AT+CONN"      //16.连接远端设备（主模式指令） 使用序号
					,"AT+CONA"      //17.连接远端设备（主模式指令） 使用mac地址
					,"AT+BAND"       //18.绑定设备（主模式指令）
					,"AT+CLRBAND"    //19.取消绑定（主模式指令）
					,"AT+DISC"       //20.断开连接（主模式指令）
					,"AT+DEFAULT"   //21.恢复出厂值（500ms 恢复出厂参数）
					,"AT+HELP"      //22.帮助
					,"AT+PWRM"	     //23.开机睡眠或开机唤醒指令
					};


					
					
					
					
//发送模块测试指令					
void get_bt_test_status(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[0], strlen((const char*)bt_cmd[0]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	



//发送模块软件复位					
void bt_test_reset(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[11], strlen((const char*)bt_cmd[11]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	


//发送模块查询版本指令					
void get_bt_test_version(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[1], strlen((const char*)bt_cmd[1]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	


//发送模块查询地址指令					
void get_bt_lvaddr(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[2], strlen((const char*)bt_cmd[2]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	

//发送模块查询地址指令					
void get_bt_name(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[3], strlen((const char*)bt_cmd[3]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}


//发送模块查询地址指令					
void set_bt_name(const uint8_t* name)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[3], strlen((const char*)bt_cmd[3]));
	Uart_Tx_String(TOCPU_COM_NUM, name, strlen((const char*)name));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}


//发送模块查询地址指令					
void set_bt_sleep(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[13], strlen((const char*)bt_cmd[13]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}



//蓝牙数据接收处理任务，10ms调用一次。
void bt_recv_task(void)
{
	
}


