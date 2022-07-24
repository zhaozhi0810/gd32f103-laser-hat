
#include "includes.h"


// definition of the commands te WT588D understands
#define WT588D_STARTLOOPPLAY    0xF2
#define WT588D_STOPLOOPPLAY     0xFE
#define WT588D_IOEXTON          0xF5
#define WT588D_IOEXTOFF         0xF6
#define WT588D_MIN_VOLUME       0xE0
#define WT588D_MAX_VOLUME       0xE7

#define MAX_NUM_OF_PLAYLISTS 30   //最多的音频数 芯片最多220，根据实际情况修改


/*
pin7 P01 K2/DATA 按键/三线数据输入脚       PB1
pin8 P02 K3/CS 按键/三线片选输入脚 		  PB0
pin9 P03 K4/CLK/DATA 按键/三线时钟/一线数据输入脚    PC5 上升沿锁存数据
*/
static uint8_t g_paly_num = 0;   //正在播放的序号
static uint8_t g_volume = 0xE0;     //当前音量值

// initialze pins
void wt588d_init(void) {
		//时钟使能
	rcu_periph_clock_enable(RCU_GPIOB);	
	rcu_periph_clock_enable(RCU_GPIOC);	

	//0 -> data  1 -> CS
	gpio_init(GPIOB, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_0 | GPIO_PIN_1);	
	gpio_bit_set(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);
	
	//clk
	gpio_init(GPIOC, GPIO_MODE_OUT_PP, GPIO_OSPEED_2MHZ, GPIO_PIN_5);	
	gpio_bit_set(GPIOC, GPIO_PIN_5);

}


static inline void wt588d_cs_low(void)
{
	gpio_bit_set(GPIOB, GPIO_PIN_1);
}

static inline void wt588d_cs_high(void)
{
	gpio_bit_set(GPIOB, GPIO_PIN_1);
}


static inline void wt588d_dat_low(void)
{
	gpio_bit_set(GPIOB, GPIO_PIN_0);
}

static inline void wt588d_dat_high(void)
{
	gpio_bit_set(GPIOB, GPIO_PIN_0);
}


static inline void wt588d_clk_low(void)
{
	gpio_bit_set(GPIOC, GPIO_PIN_5);
}

static inline void wt588d_clk_high(void)
{
	gpio_bit_set(GPIOC, GPIO_PIN_5);
}



static void wt588d_sendCommand(uint8_t command){
    
    // a 5ms reset indicates start of new command
    wt588d_cs_low();
    Delay1ms(10);

    wt588d_clk_low();
	Delay1ms(1);
	
	//根据资料，发送的周期在300us-1ms之间，推荐300us
    for(uint8_t  i = 0; i < 8; i++)  {
        if(command & (1<<i))
			wt588d_dat_high();
		else
			wt588d_dat_low();
		Delay1us(150);
		wt588d_clk_high();
		Delay1us(150);
		wt588d_clk_low();
		Delay1us(50);
    } //end for
    
    wt588d_cs_high();  // release the chip select
    Delay1ms(1);
}


// plays the playlist with in number give as arguement
// the chip supports 220 playlists
// The chips starts playing by simple receivung the number of
// the playlist from 0 to 219
void wt588d_playSound(uint8_t playListNumber){
    
    if (playListNumber>MAX_NUM_OF_PLAYLISTS)
        return;
    else {
		set_MAX9700_enable();   //音频芯片使能
        wt588d_sendCommand(playListNumber);
		g_paly_num = playListNumber;   //记录当前的编号
        return;
    }
}


void wt588d_playNextSound(void)
{
	wt588d_playSound(g_paly_num+1);
}

void wt588d_playLastSound(void)
{
	wt588d_playSound(g_paly_num-1);
}


// sets the sound volume
void wt588d_setVolume(uint8_t volume) {
    
    if (volume<= WT588D_MAX_VOLUME && volume >= WT588D_MIN_VOLUME)
    {    
		wt588d_sendCommand (volume);
		g_volume = volume;
	}
    return;
}


void wt588d_setVolume_Acc(void)
{
	wt588d_setVolume(g_volume+1);
}

void wt588d_setVolume_Dec(void)
{
	wt588d_setVolume(g_volume-1);
}

// starts continiously looping the sound that is currently played
void wt588d_startLoopSound(void) {
    
    wt588d_sendCommand (WT588D_STARTLOOPPLAY);
    
    return;
}

// stops continiously looping the sound that is currently played
void wt588d_stopLoopSound(void) {
    
    wt588d_sendCommand (WT588D_STOPLOOPPLAY);
    
    return;
}


