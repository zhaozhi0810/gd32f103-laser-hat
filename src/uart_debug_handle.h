

#ifndef UART_DEBUG_HANDLE_H
#define UART_DEBUG_HANDLE_H

extern uint8_t debug_ir_recv_mode;    //调试ir接收模式，由串口命令控制
extern uint8_t charging_enable_start_laser;  //充电模式下允许开机，1为允许


void Com_Debug_Rne_Int_Handle(void);

void Com_Debug_Idle_Int_Handle(void);

#endif
