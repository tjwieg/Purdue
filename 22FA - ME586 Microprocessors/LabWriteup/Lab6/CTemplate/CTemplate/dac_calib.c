/**
  ******************************************************************************
  * @file    dac_calib.c for ME586 Lab 6
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-17
  * @brief   Output increase internal DAC output by 16 counts with each keypress
  ******************************************************************************
  */

#include "ME586.h"
extern void WaitForKeypress();
extern void initports(int);
extern short digin();
extern void digout(short);
extern void shownum(int);

void dac_calib() {
		short counter=0;
		initports(0xF00);
		initcom();
    while (1) {
        WaitForKeypress();
        counter = counter + 16;
				if(counter>255){
					counter=255;
				}
				digout(counter);
				shownum(counter);
	}
}		
