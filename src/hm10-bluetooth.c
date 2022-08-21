
#include "includes.h"
#include <string.h>

/*
	��������  PA2��3
	
	��������֮��������͸����ATָ����Ч
	
CC41��ATָ���������ģ�
ָ��ִ�Сд��
ָ�����Ҫ���ϻ���Ҳ���ǡ�\r\n�������Ǽ��ϻ�û�з�Ӧ����
���ò���ֱ�Ӹ���ָ����棬��at+role1�����ǽ�ģ������Ϊ������

HM-10��ATָ���������ģ�
ָ��ȫ��Ҫ��д��
ָ�����Ҫ���ϻ��С�\r\n�����л��еĻ�ģ���û�з�Ӧ����
��ѯ��������Ҫ���ϡ�?������AT+ROLE?�ǲ�ѯ��ǰģ��������(Master)���Ǵӻ�(Slave)��
���ò���ֱ�Ӹ���ָ����棬��AT+ROLE1�����ǽ�ģ������Ϊ������

HC-08��ATָ���������ģ�
ָ��ȫ��Ҫ��д��
ָ�����Ҫ���ϻ��С�\r\n�����л��еĻ�ģ���û�з�Ӧ����
��ѯ��������Ҫ���ϡ�=?������AT+ROLE=?�ǲ�ѯ��ǰģ��������(Master)���Ǵӻ�(Slave)��
���ò�����ָ�����Ҫ���ϡ�=������AT+ROLE=M�����ǽ�ģ������Ϊ������

HC-05��ATָ�����ǻ�����HM-10һ����������Ҫ����ϻ��С�\r\n����


��HM-10��AT+DISC��ʾɨ�裬����HC-05���Ǳ�ʾ�Ͽ����ӡ�
HM-10����������֮���ٷ���AT��ָ����Ӿͻ��Զ��Ͽ���
HC-05�����á�AT+DISC����
���ƻ�Ҫ���ֲ�Ϊ׼������



HM-10ģ���3�ֹ���ģʽ��HM-10ģ�鹦�ܷǳ�ǿ�����ļ����ܽ���PIO���������������ɼ�������Ϊ����������PIO�Ĺ���״̬����һ��ָ����AT+MODE��
AT+MODE0: ͸��ģʽ������Ĭ��ֵ�����ǵ���һ������ˢ��ģ��Ҫ��AT+MODE1����ΪĬ��ֵ������ATָ��û�з�Ӧ����
1.���ǲ������ATָ�ֱ�ӽ��յ�������ת����

AT+MODE1: PIO �ɼ�+Զ��+͸��
1.PIO2~3�������ܽ�Ϊ�����
2.PIO4~11��8���ܽ�Ϊ���룻���ɼ����ݣ�����ɼ�ʲô���ݣ������в鿴HM-10��˵���飩
3.����ATָ�
4.�ǡ�AT����ͷ���ַ���ֱ��ת����

AT+MODE2��͸��+Զ��ģʽ
1.PIO2~11�������ܽ�Ϊ�����
2.����ATָ�
3.�ǡ�AT����ͷ���ַ���ֱ��ת����

*/

const uint8_t* bt_cmd_end = "\r\n";

const uint8_t* bt_cmd[] = {   "AT"        // 1.����ָ��
					,"AT+VERSION"      //2.��ȡ����汾��
					,"AT+LADDR"      //3.��ȡģ��������ַ
					,"AT+NAME"      //4.����/��ѯ�豸����
					,"AT+PIN"      //5.����/��ѯ-�����
					,"AT+TYPE"      //6.����/��ѯ-ģ���Ȩ��������
					,"AT+BAUD"      //7.����/��ѯ-���ڲ�����
					,"AT+ADVI"      //8.����/��ѯ-�㲥���
					,"AT+POWE"      //9.����ģ�鷢�书��
					,"AT+UUID"      //10.����/��ѯ-Service UUID
					,"AT+CHAR"     //11.����/��ѯ-Characteristic
					,"AT+RESET"    //12.���������500ms ��������
					,"AT+ROLE"     //13.����/��ѯ-��/��ģʽ
					,"AT+SLEEP"    //14.����͹���ģʽ�������ɱ�������
					,"AT+INQ"      //15.���������豸����ģʽָ�
					,"AT+CONN"      //16.����Զ���豸����ģʽָ� ʹ�����
					,"AT+CONA"      //17.����Զ���豸����ģʽָ� ʹ��mac��ַ
					,"AT+BAND"       //18.���豸����ģʽָ�
					,"AT+CLRBAND"    //19.ȡ���󶨣���ģʽָ�
					,"AT+DISC"       //20.�Ͽ����ӣ���ģʽָ�
					,"AT+DEFAULT"   //21.�ָ�����ֵ��500ms �ָ�����������
					,"AT+HELP"      //22.����
					,"AT+PWRM"	     //23.����˯�߻򿪻�����ָ��
					};


					
					
					
					
//����ģ�����ָ��					
void get_bt_test_status(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[0], strlen((const char*)bt_cmd[0]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	



//����ģ�������λ					
void bt_test_reset(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[11], strlen((const char*)bt_cmd[11]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	


//����ģ���ѯ�汾ָ��					
void get_bt_test_version(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[1], strlen((const char*)bt_cmd[1]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	


//����ģ���ѯ��ַָ��					
void get_bt_lvaddr(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[2], strlen((const char*)bt_cmd[2]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}	

//����ģ���ѯ��ַָ��					
void get_bt_name(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[3], strlen((const char*)bt_cmd[3]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}


//����ģ���ѯ��ַָ��					
void set_bt_name(const uint8_t* name)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[3], strlen((const char*)bt_cmd[3]));
	Uart_Tx_String(TOCPU_COM_NUM, name, strlen((const char*)name));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}


//����ģ���ѯ��ַָ��					
void set_bt_sleep(void)
{
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd[13], strlen((const char*)bt_cmd[13]));
	Uart_Tx_String(TOCPU_COM_NUM, bt_cmd_end, strlen((const char*)bt_cmd_end));
}



//�������ݽ��մ�������10ms����һ�Ρ�
void bt_recv_task(void)
{
	
}


