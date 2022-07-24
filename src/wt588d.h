

#ifndef __WT588D_H__
#define __WT588D_H__


// initialze pins
void wt588d_init(void);
void wt588d_playSound(uint8_t playListNumber);   //播放指定的音乐
// sets the sound volume
void wt588d_setVolume(uint8_t volume);  //调节音量 0x0xE0 ~ 0xE7


void wt588d_startLoopSound(void);  //开始循环播放
void wt588d_stopLoopSound(void);   //关闭循环播放


void wt588d_setVolume_Acc(void);  //音量增加

void wt588d_setVolume_Dec(void);  //音量减小

void wt588d_playNextSound(void);  //下一首

void wt588d_playLastSound(void);  //上一首


#endif
