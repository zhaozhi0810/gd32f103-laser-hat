

#ifndef UART_DEBUG_HANDLE_H
#define UART_DEBUG_HANDLE_H

extern uint8_t debug_ir_recv_mode;    //����ir����ģʽ���ɴ����������
extern uint8_t charging_enable_start_laser;  //���ģʽ����������1Ϊ����


void Com_Debug_Rne_Int_Handle(void);

void Com_Debug_Idle_Int_Handle(void);

#endif
