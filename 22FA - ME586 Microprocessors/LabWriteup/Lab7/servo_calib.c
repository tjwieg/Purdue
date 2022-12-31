#include "ME586.h"

int main(void) {
	int value = 1000;
	initcom();
	initadc();
	initdac();
	
	printf("DAC:\t ADC:\n\r");
	
	while(1){
		WaitForKeypress();
		d_to_a(0, value);
		shownum(value);
		putchar('\t');
		
		WaitForKeypress();
		shownum(a_to_d(1));
		printf("\n\r");
		
		value=value+200;
		if(value > 3000){
			break;
		}
	}
}