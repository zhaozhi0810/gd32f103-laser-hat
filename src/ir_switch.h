
#ifndef __IR_SWITCH_H__
#define __IR_SWITCH_H__

#define IR_DETECT_USE_IRQ   //������ʹ���жϷ�ʽ


void ir_pwm_init(void);

//������ʱ������38KhzƵ�ʣ���ʱ������������Ϊ1�������ƽΪ0
void ir_send_high(void);

//�رն�ʱ�����ٷ���38KhzƵ�ʣ���ʱ������������Ϊ0�������ƽΪ1
void ir_send_low(void);


//����������ţ�����Ϊ�жϷ�ʽ��
void ir_detect_init(void);


// 100ms ����1��
//ϵͳ�����ż�⣬�������Ͳ��ü���ˣ�����
void ir_irq9_detect_task(void);


//�ر�irq��⣬�ػ�ʱ�Ͳ���Ҫ�ˡ�
void ir_detect_off(void);
#endif

