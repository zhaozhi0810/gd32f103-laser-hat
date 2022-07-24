

#ifndef __WT588D_H__
#define __WT588D_H__


// initialze pins
void wt588d_init(void);
void wt588d_playSound(uint8_t playListNumber);   //����ָ��������
// sets the sound volume
void wt588d_setVolume(uint8_t volume);  //�������� 0x0xE0 ~ 0xE7


void wt588d_startLoopSound(void);  //��ʼѭ������
void wt588d_stopLoopSound(void);   //�ر�ѭ������


void wt588d_setVolume_Acc(void);  //��������

void wt588d_setVolume_Dec(void);  //������С

void wt588d_playNextSound(void);  //��һ��

void wt588d_playLastSound(void);  //��һ��


#endif
