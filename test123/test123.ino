#include <EEPROM.h>
#include <TimerOne.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <avr/eeprom.h>

#define INTERVAL  13000

#define FLAG_Volt_bm		((unsigned char)(1<<0)) //
#define FLAG_Time_bm		((unsigned char)(1<<1)) //
#define FLAG_UpSwitch_bm		((unsigned char)(1<<2)) //
#define FLAG_DownSwith_bm			((unsigned char)(1<<3)) //
#define FLAG_ModeSwith_bm			((unsigned char)(1<<4)) //
#define FLAG_PwrSwith_bm			((unsigned char)(1<<5)) //
#define FLAG_Modename_bm		((unsigned char)(1<<6)) //
#define FLAG_BZ_bm			((unsigned char)(1<<7)) //

#define N_CHAT_THRESH			 0 // 2msec * 16 = 32msec
#define N_SLEEP_TIME			90
#define N_POWER_ONOFF			50
#define N_BZ_period             100
#define N_Volt_period			20
#define N_modename_period		120 // 0.4sec 0.1 sec = 30 300 = 1 sec 18000 = 1min
#define N_5MIN					11690// 38.2 = 1sec 2300 = 1min 11690 = 5min
#define N_Temperature_period    10 // 5
#define N_Temperature_freq		30
#define N_button_push			7
#define N_5SEC					200

enum sysmode { BZOFF, BZON };
enum bzsmode { OFF, ON };
enum segmode { Appear_off,volt, Appear_CurrentTemper, Appear_HighTemper, Appear_LowTemper, Appear_Time};

#define SG1  PB6
#define SG2  PB7
#define SG3  PD5
#define SG4  PD6
#define SG5  PD7
#define SG6  PB0
#define SG7  PB1
#define SG8  PB2

#define GR1d  PD2
#define GR2d  PD3
#define GR3d  PD4

#define LAMP_OUT				PC0
#define PWR_TOUCH_IN			PC2
#define MODE_TOUCH_IN			PC3
#define UP_TOUCH_IN				PC4
#define DOWN_TOUCH_IN			PC5

volatile unsigned char flag 	= 	0;
volatile unsigned char flag_Swith 	= 	0;

volatile unsigned char tim_base	=	0;

volatile unsigned char touch_chat	=	0;
volatile unsigned char touch_prev = 0;
volatile unsigned char touch_curr = 0;
volatile unsigned char touch_data = 0;

volatile unsigned char mode_touch_chat	=	0;
volatile unsigned char mode_touch_prev = 0;
volatile unsigned char mode_touch_curr = 0;
volatile unsigned char mode_tim_base	=	0;

volatile unsigned char up_touch_chat	=	0;
volatile unsigned char up_touch_prev = 0;
volatile unsigned char up_touch_curr = 0;
volatile unsigned char up_tim_base	=	0;

volatile unsigned char down_touch_chat	=	0;
volatile unsigned char down_touch_prev = 0;
volatile unsigned char down_touch_curr = 0;
volatile unsigned char down_tim_base	=	0;

volatile uint16_t temper_Doing = 0;
volatile uint16_t modename_Doing = 0;
volatile uint16_t  time_Doing = 0;
volatile unsigned char time_Check = 0;
volatile unsigned char Time_tmp	=	0;

volatile uint16_t bz_doing	=	0;
volatile uint16_t bz_point	=	0;

volatile uint16_t hightemper_Doing = 0;
volatile uint16_t lowtemper_Doing = 0;

volatile unsigned char sys_mode = 0;
volatile unsigned char bz_mode = 0;
volatile unsigned char segment_mode = 0;
volatile unsigned char switch_mode = 0;

volatile int16_t hightemper; 
volatile int16_t lowtemper;
volatile int16_t timesetvalue;
volatile int16_t EEPROMtmp;
volatile unsigned char segmentdo = 1; 

volatile int power_touch_chat = 0;
volatile float Volt_Detect = 0;

volatile float CurrentTemperature= 0;
volatile float Vo;
volatile float tem;

volatile unsigned char volt_tmp = 0;
volatile unsigned char fivesec_tmp = 0;

volatile int16_t* HighTaddr = 1;
volatile int16_t* lowTaddr = 10;
volatile int16_t* timeTaddr = 20;
volatile int16_t* EEPROMaddr = 30;

void Set_7segment(int digit,int number);
void Set_timeSegment(int digit,int number);
void HandleTouchKeyInput(void);
void BZ_Control(void);
void Temperature_appear(void);
void Volt_appear(void);
void DoingTime(void);
void fivesec_return(void);

void Set_timeSegment(int digit,int number)// 1 2 3
{
	
		switch(digit)
		{
			case 1:
			switch(number)
			{
				case 0: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8); // 0
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 1: PORTB = (1 << SG1) | (0 << SG2) | (1 << SG6) | (1 << SG7) | (1 << SG8); // 1
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 2: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8); // 2
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (1 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 3: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8); // 3
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 4: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // 4
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 5: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // 5
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 6: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 6
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 7: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8);// 7
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 8: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 8
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 9: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 9
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
			}
			case 2:
			switch(number)
			{
				case 0: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8); // 0
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 1: PORTB = (1 << SG1) | (0 << SG2) | (1 << SG6) | (1 << SG7) | (1 << SG8); // 1
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 2: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8); // 2
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (1 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 3: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8); // 3
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 4: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // 4
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 5: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // 5
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 6: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 6
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 7: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8);// 7
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 8: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 8
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 9: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 9
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
			}
			case 3:
			switch(number)
			{
				case 12: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// H
				PORTD = (0 << SG3) | (1 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				break;
			}// H(12)
		}
}
	
	

void Set_7segment(int digit,int number) // 0 -> HT(10),LT(11) 1 -> -(10) 3 2 1
{
	switch(digit)
	{
		
			case 0:
			switch(number)
			{
				case 0: PORTD = (0 << GR1d) | (0 << GR2d) | (0 << GR3d); break;
				case 10: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // HT
				PORTD = (0 << SG3) | (1 << SG4) | (0 << SG5) | (0 << GR1d) | (1 << GR2d) | (0 << GR3d);
			
				PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (1 << SG3) | (0 << SG4) | (0 << SG5) | (0 << GR1d) | (0 << GR2d) | (1 << GR3d);
				break;
				case 11: PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8); // LT
				PORTD = (1 << SG3) | (0 << SG4) | (0 << SG5) | (0 << GR1d) | (1 << GR2d) | (0 << GR3d);
			
				PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (1 << SG3) | (0 << SG4) | (0 << SG5) | (0 << GR1d) | (0 << GR2d) | (1 << GR3d);
				break;
			}
			break;
		
			case 1:
			switch(number)
			{
				case 0: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8); // 0
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 1: PORTB = (1 << SG1) | (0 << SG2) | (1 << SG6) | (1 << SG7) | (1 << SG8); // 1
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 2: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8); // 2
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (1 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 3: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8); // 3
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 4: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // 4
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 5: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // 5
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 6: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 6
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 7: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8);// 7
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 8: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 8
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 9: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);// 9
				PORTD = (1 << GR1d) | (0 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;			
			}
			break;
		
			case 2:
			switch(number)
			{
				case 0: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 1: PORTB = (1 << SG1) | (0 << SG2) | (1 << SG6) | (1 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 2: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (1 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 3: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 4: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 5: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 6: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 7: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 8: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 9: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (0 << SG8);
				PORTD = (0 << GR1d) | (1 << GR2d) | (0 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
			}
			break;
		
			case 3:
			switch(number)
			{
				case 0: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 1: PORTB = (1 << SG1) | (0 << SG2) | (1 << SG6) | (1 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 2: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (1 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 3: PORTB = (0 << SG1) | (0 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 4: PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 5: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 6: PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 7: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (1 << SG4) | (1 << SG5);
				break;
				case 8: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (0 << SG5);
				break;
				case 9: PORTB = (0 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8);
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (0 << SG3) | (0 << SG4) | (1 << SG5);
				break;
				case 10: PORTB = (1 << SG1) | (1 << SG2) | (1 << SG6) | (0 << SG7) | (1 << SG8);// -
				PORTD = (0 << GR1d) | (0 << GR2d) | (1 << GR3d) | (1 << SG3) | (1 << SG4) | (1 << SG5);
				break;
			}
			break;
		
	}
	
	return;
}

void setup()
{
	DDRB |= (1<<PB6) | (1<<PB7) | (1<<PB2) | (1<<PB0) | (1<<PB1) | (1<<PB3);
	DDRC |= (1<<PC0) | (0<<PC1);        
	DDRD |= (1<<PD2) | (1<<PD3) | (1<<PD4) | (1<<PD5) | (1<<PD6) | (1<<PD7); //GR -> 1 << LED on SG -> 0 << LED on
	//       GR3         GR2        GR1         SG3         SG4      SG5
	PORTC |= (1<<LAMP_OUT); // LAMP_OUT = PC0
	PORTD |= (0<<PD2) | (0<<PD3) | (0<<PD4);
	
	Timer1.initialize(INTERVAL);
	Timer1.attachInterrupt(timeisr);
	Serial.begin(9600);	
	
	Vo = analogRead(A7);
	tem = analogRead(A1);
	
	Volt_Detect = tem * 5.0 / 1024.0/ 0.2;
	
	float R1 = 10000;
	float logR2, R2, T;
	float c1 =24.95790399e-03, c2 = -39.26563356e-04, c3 = 166.9926902e-07;
	R2 = R1 * (1024.0 / (float)Vo - 1.0);
	logR2 = log(R2);
	T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
	CurrentTemperature = T - 273.15;
	
	hightemper = eeprom_read_word(HighTaddr);
	lowtemper = eeprom_read_word(HighTaddr);
	timesetvalue = eeprom_read_word(timeTaddr);
	EEPROMtmp = eeprom_read_word(EEPROMaddr);
	
	if(EEPROMtmp != 1)
	{
		hightemper = 750;
		lowtemper = 745;
		timesetvalue = 9;
		EEPROMtmp = 1;
		
		eeprom_write_word(HighTaddr,hightemper);
		eeprom_write_word(lowTaddr,lowtemper);
		eeprom_write_word(timeTaddr,timesetvalue);
		eeprom_write_word(EEPROMaddr,EEPROMtmp);
	}
	
}

void loop()
{
	tem = analogRead(A1); // volt
	
	int volt_tmp = Volt_Detect * 10.0;
	
	if(segment_mode == Appear_off)
		{
			bz_mode = BZOFF;
			PORTD &= ~(1<<PD2);
			PORTD &= ~(1<<PD3);
			PORTD &= ~(1<<PD4);
		}
		
	if(segment_mode == volt)
		{
			if(segmentdo)
			{	
				int m = volt_tmp % 1000;
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(1,m % 10);
				_delay_ms(1);
				m /= 10;
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(2,m % 10);
				_delay_ms(1);
				m /= 10;
				if(bz_mode == BZON && bz_point == 0)
				{
					PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8) | (1 << PB3);
					PORTD = (0 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				}
				else Set_7segment(3,m % 10);
				_delay_ms(1);
			}
			else 
			{
				PORTD &= ~(1<<PD2);
				PORTD &= ~(1<<PD3);
				PORTD &= ~(1<<PD4);
			}
		}
		
	if(segment_mode == Appear_CurrentTemper)
		{
			if(Vo != 0)
			{
				float Tempertmp = CurrentTemperature * 10.0;
				int Tm = (int)Tempertmp % 1000;
				
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
					}
				else  Set_7segment(1,Tm % 10);
				Tm /= 10;
				_delay_ms(1);
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(2,Tm % 10);
				Tm /= 10;
				_delay_ms(1);
				if(bz_mode == BZON && bz_point == 0)
				{
					PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8) | (1 << PB3);
					PORTD = (0 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				}
				else Set_7segment(3,Tm % 10);
				_delay_ms(1);
			}
			else 
			{
				PORTD = (0<<GR1d) | (0<<GR2d) | (0<<GR3d);
				PORTC = (1<<LAMP_OUT);
			}
		}
		
	if(segment_mode == Appear_HighTemper)
	{
		if(flag & FLAG_Modename_bm )
		{
			if(++modename_Doing > N_modename_period) 
			{
				flag &= ~(FLAG_Modename_bm);
				modename_Doing = 0;
			}
			
				PORTB = (1 << SG1) | (0 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // H
				PORTD = (0 << SG3) | (1 << SG4) | (0 << SG5) | (0 << GR1d) | (1 << GR2d) | (0 << GR3d);
				_delay_ms(1);
				PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // E
				PORTD = (1 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				_delay_ms(1);
		
		}
		else
		{
			int tmp = hightemper;
			tmp = tmp - 200;
			if(tmp < 0)
				{
				tmp = -1 * tmp;
				if(bz_mode == BZON && bz_point == 0) {
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(3,10);
				tmp /= 10;
				_delay_ms(1);
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(1,tmp % 10);
				_delay_ms(1);
				tmp /= 10;
				if(bz_mode == BZON && bz_point == 0)
				{
					PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8) | (1 << PB3); 
					PORTD = (0 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				}
				else Set_timeSegment(2,tmp % 10);
				_delay_ms(1);
			}
			else
			{
				tmp = tmp % 1000;
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(1,tmp % 10);
				_delay_ms(1);
				tmp /= 10;
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(2,tmp % 10);
				_delay_ms(1);
				tmp /= 10;
				if(bz_mode == BZON && bz_point == 0)
				{
					PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8) | (1 << PB3);
					PORTD = (0 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				}
				else Set_7segment(3,tmp % 10);
				_delay_ms(1);
			}
		}
	}
	
	if(segment_mode == Appear_LowTemper)
	{
		if(flag & FLAG_Modename_bm)
		{
			if(++modename_Doing > N_modename_period)
			{
				flag &= ~(FLAG_Modename_bm);
				modename_Doing = 0;
			}
				PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (1 << SG7) | (1 << SG8); // L
				PORTD = (1 << SG3) | (0 << SG4) | (0 << SG5) | (0 << GR1d) | (1 << GR2d) | (0 << GR3d);
				_delay_ms(1);
				PORTB = (0 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8); // E
				PORTD = (1 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				_delay_ms(1);
		}
		else
		{
			int tmpL = lowtemper;
			tmpL = tmpL - 200;
			if(tmpL < 0)
			{
				tmpL = -1 * tmpL;
				if(bz_mode == BZON && bz_point == 0) {
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(3,10);
				tmpL /= 10;
				_delay_ms(1);
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(1,tmpL % 10);
				_delay_ms(1);
				tmpL /= 10;
				if(bz_mode == BZON && bz_point == 0)
				{
					PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8) | (1 << PB3);
					PORTD = (0 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				}
				else Set_7segment(2,tmpL % 10);
				_delay_ms(1);
			}
			else
			{
				tmpL = tmpL % 1000;
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(1,tmpL % 10);
				_delay_ms(1);
				tmpL /= 10;
				if(bz_mode == BZON && bz_point == 0){
					PORTB |= (1 << PB3);
					PORTD |= (0<<GR3d) | (0 << GR1d) | (0 << GR2d);
				}
				else Set_7segment(2,tmpL % 10);
				_delay_ms(1);
				tmpL /= 10;
				if(bz_mode == BZON && bz_point == 0)
				{
					PORTB = (1 << SG1) | (1 << SG2) | (0 << SG6) | (0 << SG7) | (1 << SG8) | (1 << PB3);
					PORTD = (0 << SG3) | (0 << SG4) | (0 << SG5) | (1 << GR1d) | (0 << GR2d) | (0 << GR3d);
				}
				else Set_7segment(3,tmpL % 10);
				_delay_ms(1);
			}
		}
	}
	
	if(segment_mode == Appear_Time)
	{
		unsigned char tmpT = timesetvalue + 1;
		Set_timeSegment(3,12);//1 2 3
		_delay_ms(1);
		Set_timeSegment(2,tmpT % 10);
		
		_delay_ms(1);
		tmpT /= 10;
		Set_7segment(3,tmpT);// 3 2 1
		_delay_ms(1);
	}
	
	 if(volt_tmp < 95) 
	 {
		 PORTD &= ~(1<<PD2);
		 PORTD &= ~(1<<PD3);
		 PORTD &= ~(1<<PD4);
		 bz_mode = BZOFF;
		 bz_point = 1;
		 sys_mode = OFF;
		 segment_mode = Appear_off;
		 PORTC |= (1<<LAMP_OUT);
		 PORTB &= ~(1<<PB3);
	 }
	 else if(volt_tmp >= 95 && volt_tmp < 96 ) bz_mode = BZON;
	 else if(volt_tmp > 97) bz_point = 0;
	 
	
	BZ_Control();
	HandleTouchKeyInput();
	DoingTime();
	fivesec_return();
  }

void DoingTime(void)
{
	if(!(flag & FLAG_Time_bm)) return;
	else flag &= ~FLAG_Time_bm;
	
	
	if(temper_Doing++ > N_Temperature_period)
	{
		Volt_Detect = tem * 5.0 / 1024.0/ 0.2;
		
		uint8_t tmp = 0;
		float Votmp = 0;
		while(tmp < 10)
		{
			Votmp += analogRead(A7);
			tmp++;
		}
		Vo = Votmp / 10.0f; // temper
		
		temper_Doing = 0;
		float R1 = 10000;
		float logR2, R2, T;
		float c1 =24.95790399e-03, c2 = -39.26563356e-04, c3 = 166.9926902e-07;
		R2 = R1 * (1024.0 / (float)Vo - 1.0);
		logR2 = log(R2);
		T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
		CurrentTemperature = T - 273.15;
	}
	
	float tmp = CurrentTemperature * 10.0;
	int Tempertmp = (int)tmp % 1000;
	
	hightemper = eeprom_read_word(HighTaddr);
	lowtemper = eeprom_read_word(lowTaddr);
	timesetvalue = eeprom_read_word(timeTaddr);
	
	if(sys_mode == ON)
		{
			if(Tempertmp >= 650)
			{
				 PORTC |= (1<<LAMP_OUT);
				 bz_mode = BZOFF;
				 bz_point = 1;
				 PORTB |= B11101111;
				 PORTD &= ~(1<<PD2);
				 PORTD &= ~(1<<PD3);
				 PORTD &= ~(1<<PD4);
				 sys_mode = OFF;
				 segment_mode = Appear_off;
			}
			else if(Tempertmp >= 600 & Tempertmp < 650)
			{
				bz_mode = BZON;
				PORTB |= (1<<PB3);
				bz_point = 0;
			}
			
			if(Time_tmp == 0)
			{
				if(time_Check >= (timesetvalue + 1) * 12) 
				{
					PORTC |= (1<<LAMP_OUT);
					bz_mode = BZOFF;
					PORTB |= B11101111;
					PORTD &= ~(1<<PD2);
					PORTD &= ~(1<<PD3);
				    PORTD &= ~(1<<PD4);
					sys_mode = OFF;
					segment_mode = Appear_off;
				}
				else if(time_Check < (timesetvalue + 1) * 12)
				{
					if(Tempertmp >= hightemper - 200)
					{
						if(++hightemper_Doing > N_Temperature_freq)
						 {
							 hightemper_Doing = 0;
							 PORTC |= (1<<LAMP_OUT); // off
						 }
					}
					else if(Tempertmp <= lowtemper - 200) 
					{
						if(++lowtemper_Doing > N_Temperature_freq)
						{
							lowtemper_Doing = 0;
							 if(Vo != 0) PORTC &= ~(1<<LAMP_OUT);// on
						}
					}
				}
			
				if(++time_Doing == N_5MIN)
				{
					time_Doing = 0;
					++time_Check;
				}
			}	
		}
	if(sys_mode == OFF) 
	{
		PORTC |= (1<<LAMP_OUT);
		bz_mode = BZOFF;
	}
	
}

void BZ_Control(void) 
{
	if(!(flag & FLAG_BZ_bm)) return;
	else flag &= ~(FLAG_BZ_bm);
	
	if(sys_mode == ON)
		{
			if(bz_mode == BZOFF) PORTB &= ~(1<<PB3);
			else if(bz_mode == BZON && bz_point == 0)
			{
				PORTB |= (1<<PB3); //bz
				if(++bz_doing > N_BZ_period)
				{
					PORTB &= ~(1<<PB3);
					bz_mode = BZOFF;
					bz_point = 1;
					bz_doing = 0;
				}
			}
		}
	else PORTB &= ~(1<<PB3);
}

void fivesec_return(void)
{
	if(!(flag_Swith & FLAG_Time_bm)) return;
	else flag_Swith &= ~(FLAG_Time_bm);
	
	if(segment_mode == Appear_HighTemper || segment_mode == Appear_LowTemper || segment_mode == Appear_Time)
		{
			if(++fivesec_tmp > N_5SEC) 
			{
				segment_mode = volt;
				fivesec_tmp = 0;
			}
		}
}

void HandleTouchKeyInput(void)
{
	//PWRSWITH
	if( flag & FLAG_PwrSwith_bm ) 
	{
		flag &= ~FLAG_PwrSwith_bm;
	
		touch_curr = PINC & (1<<PWR_TOUCH_IN);
			if(touch_curr == touch_prev)
			{
					if(++touch_chat > N_CHAT_THRESH )
				   {	
						touch_chat = 0;
				
						if( !(touch_prev & (1<<PWR_TOUCH_IN)) )
						{
							if( !(flag_Swith & FLAG_PwrSwith_bm) )
							{
								flag_Swith |= FLAG_PwrSwith_bm;
									tim_base = 0; // clear sleep-timer while key is pressed.
						
									if( ++sys_mode > ON ) sys_mode = OFF;
									switch(sys_mode)
									{
										case ON:
										segment_mode = volt;
										Time_tmp = 0;
										time_Check = 0;
										time_Doing = 0;
										bz_point = 0;
										break;
										
										case OFF:
										segment_mode = Appear_off;
										bz_mode = BZOFF;
										PORTC |= (1<<LAMP_OUT);
										segmentdo = 1;
										PORTB |= B11101111;
										
										/*
										PORTD &= ~(1<<PD2);
										PORTD &= ~(1<<PD3);
										PORTD &= ~(1<<PD4);
										*/
										break;
									}
							}
						}
				   
					else
					{
						flag_Swith &= ~(FLAG_PwrSwith_bm);
						//power_touch_chat = 0;
					}
				}
			}
			else
			{
				touch_prev = touch_curr;
				touch_chat = 0;
				//power_touch_chat = 0;
			}
	}
	
	//MODESWITH
	if( flag & FLAG_ModeSwith_bm )
	{
		flag &= ~FLAG_ModeSwith_bm;
		
		mode_touch_curr = PINC & (1<<MODE_TOUCH_IN);
		if(mode_touch_curr == mode_touch_prev)
		{
			if(++mode_touch_chat > N_CHAT_THRESH )
			{
				mode_touch_chat = 0;
				
				if( !(mode_touch_prev & (1<<MODE_TOUCH_IN)) )
				{
					if( !(flag_Swith & FLAG_ModeSwith_bm) )
					{
						flag_Swith |= FLAG_ModeSwith_bm;
						mode_tim_base = 0; // clear sleep-timer while key is pressed.
						
						if( ++segment_mode > Appear_Time ) segment_mode = volt;
						switch(segment_mode)
						{
							case volt:
								flag &= ~FLAG_Modename_bm;
								PORTD &= ~(1<<PD2);
								PORTD &= ~(1<<PD3);
								PORTD &= ~(1<<PD4);
							break;
							
							case Appear_CurrentTemper:
								flag &= ~FLAG_Modename_bm;
								PORTD &= ~(1<<PD2);
								PORTD &= ~(1<<PD3);
								PORTD &= ~(1<<PD4);
							break;
							
							case Appear_HighTemper:
								flag |= FLAG_Modename_bm;
								fivesec_tmp = 0;
								PORTD &= ~(1<<PD2);
								PORTD &= ~(1<<PD3);
								PORTD &= ~(1<<PD4);
							break;
							
							case Appear_LowTemper:
								flag |= FLAG_Modename_bm;
								fivesec_tmp = 0;
								PORTD &= ~(1<<PD2);
								PORTD &= ~(1<<PD3);
								PORTD &= ~(1<<PD4);
							break;
							
							case Appear_Time:
								flag &= ~FLAG_Modename_bm;
								fivesec_tmp = 0;
								PORTD &= ~(1<<PD2);
								PORTD &= ~(1<<PD3);
								PORTD &= ~(1<<PD4);
							break;
							
						}
						
					}
				}
				
				else
				{
					flag_Swith &= ~(FLAG_ModeSwith_bm);
					//power_touch_chat = 0;
				}
			}
		}
		else
		{
			mode_touch_prev = mode_touch_curr;
			mode_touch_chat = 0;
			//power_touch_chat = 0;
		}
	}
	
	//UPSWITH
	if( flag & FLAG_UpSwitch_bm )
	{
		flag &= ~FLAG_UpSwitch_bm;
		
		up_touch_curr = PINC & (1<<UP_TOUCH_IN);
		if(up_touch_curr == up_touch_prev)
		{
			if(++up_touch_chat > N_CHAT_THRESH )
			{
				up_touch_chat = 0;
				
				if( !(up_touch_prev & (1<<UP_TOUCH_IN)) )
				{
					if(++up_tim_base > N_button_push)
						{
							up_tim_base = 0;
							flag_Swith &= ~(FLAG_UpSwitch_bm);
						}
					if( !(flag_Swith & FLAG_UpSwitch_bm) )
					{
						flag_Swith |= FLAG_UpSwitch_bm;
						up_tim_base = 0; // clear sleep-timer while key is pressed.
						
						switch(segment_mode)
						{
							case volt:
							
							break;
							
							case Appear_HighTemper:
							fivesec_tmp = 0;
							hightemper = eeprom_read_word(HighTaddr);
							if(hightemper <= 190) hightemper = hightemper + 10;
							else
							{
								++hightemper;
								hightemper = hightemper % 801;
							}
							eeprom_update_word(HighTaddr,hightemper);
							break;
							
							case Appear_LowTemper:
							fivesec_tmp = 0;
							lowtemper = eeprom_read_word(lowTaddr);
							if(lowtemper <= 190) lowtemper = lowtemper + 10;
							else
							{
								++lowtemper;
								lowtemper = lowtemper % 801;
							}
							eeprom_update_word(lowTaddr,lowtemper);
							break;
							
							case Appear_Time:
							fivesec_tmp = 0;
							time_Check = 0; 
							Time_tmp = 0;
							time_Doing = 0;
							timesetvalue = eeprom_read_word(timeTaddr);
							++timesetvalue;
							timesetvalue = timesetvalue % 12;
							eeprom_update_word(timeTaddr,timesetvalue);
							break;
							
						}
						
					}
				}
				
				else
				{
					flag_Swith &= ~(FLAG_UpSwitch_bm);
					//power_touch_chat = 0;
				}
			}
		}
		else
		{
			up_touch_prev = up_touch_curr;
			up_touch_chat = 0;
			//power_touch_chat = 0;
		}
	}
	
	
	//DOWNSWITH
	if( flag & FLAG_DownSwith_bm )
	{
		flag &= ~FLAG_DownSwith_bm;
		
		down_touch_curr = PINC & (1<<DOWN_TOUCH_IN);
		if(down_touch_curr == down_touch_prev)
		{
			if(++down_touch_chat > N_CHAT_THRESH )
			{
				down_touch_chat = 0;
				
				if( !(down_touch_prev & (1<<DOWN_TOUCH_IN)) )
				{
					if(++down_tim_base > N_button_push)
					{
						down_tim_base = 0;
						flag_Swith &= ~(FLAG_DownSwith_bm);
					}
					if( !(flag_Swith & FLAG_DownSwith_bm) )
					{
						flag_Swith |= FLAG_DownSwith_bm;
						down_tim_base = 0; // clear sleep-timer while key is pressed.
						
						switch(segment_mode)
						{
							case volt:
							
							break;
							
							case Appear_HighTemper:
							fivesec_tmp = 0;
							hightemper = eeprom_read_word(HighTaddr);
							if(hightemper <= 200) 
							{
								hightemper = hightemper - 10;
								if(hightemper < 0) hightemper = 800;
							}
							else --hightemper;
							eeprom_update_word(HighTaddr,hightemper);
							break;
							
							case Appear_LowTemper:
							fivesec_tmp = 0;
							lowtemper = eeprom_read_word(lowTaddr);
							if(lowtemper <= 200)
							{
								lowtemper =	lowtemper - 10;
								if(lowtemper < 0) lowtemper = 800;
							}
							else --lowtemper;
							eeprom_update_word(lowTaddr,lowtemper);
							break;
							
							case Appear_Time:
							fivesec_tmp = 0;
							time_Check = 0; 
							Time_tmp = 0;
							time_Doing = 0;
							timesetvalue = eeprom_read_word(timeTaddr);
							--timesetvalue;
							if(timesetvalue < 0) timesetvalue = 11;
							eeprom_update_word(timeTaddr,timesetvalue);
							break;
							
						}
						
					}
				}
				
				else
				{
					flag_Swith &= ~(FLAG_DownSwith_bm);
					//power_touch_chat = 0;
				}
			}
		}
		else
		{
			down_touch_prev = down_touch_curr;
			down_touch_chat = 0;
			//power_touch_chat = 0;
		}
	}
}

void timeisr()
{
	flag |= FLAG_BZ_bm;
// 	if(Volt_Detect >= 9.5)
// 	{
		flag |= FLAG_PwrSwith_bm;
		if(sys_mode == ON)
		{
			flag |= FLAG_Time_bm;
			flag |= FLAG_ModeSwith_bm;
			flag |= FLAG_DownSwith_bm;
			flag |= FLAG_UpSwitch_bm;
			flag_Swith |= FLAG_Time_bm;
		}
	/*}*/
}