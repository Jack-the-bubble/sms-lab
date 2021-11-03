#include "stm32f10x.h"

#define LED1 GPIO_Pin_8
#define LED2 GPIO_Pin_9
#define LED3 GPIO_Pin_10
#define LED4 GPIO_Pin_11
#define LED5 GPIO_Pin_12
#define LED6 GPIO_Pin_13
#define LED7 GPIO_Pin_14
#define LED8 GPIO_Pin_15
#define LEDALL (LED1|LED2|LED3|LED4|LED5|LED6|LED7|LED8)

enum LED_ACTION { LED_ON, LED_OFF, LED_TOGGLE };

void LED(uint16_t led, enum LED_ACTION act);
void refreshLCD(void);
char KB2char(void);
extern unsigned char pad_result[10];

extern unsigned char bufor[10];
extern uint8_t count;

static const uint16_t V25 = 1750; // gdy V25 =1.41 V dla napi e cia odniesienia 3.3V
static const uint16_t Avg_Slope = 5; // gdy Avg_Slope =4.3 mV/C dla napi e cia odniesienia 3.3V
