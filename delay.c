#include "delay.h"
void DelayTick(void){
	if(msc > 0) --msc;
}

void Delay(unsigned int ms){
	msc = ms;
	while(msc > 0);
}
