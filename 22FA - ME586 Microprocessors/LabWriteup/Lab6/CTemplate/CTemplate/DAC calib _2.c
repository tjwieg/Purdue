/**
  ******************************************************************************
  * @file    analog_copy.c for ME586 Homework 6
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-17
  * @brief   Read analog input, output the same voltage on analog output
  ******************************************************************************
  */

#include "ME586.h"
extern void WaitForKeypress();


void dac_calib_2() {
		short counter=0;
		initcom();
		initdac();
    while (1) {
        WaitForKeypress();
        counter = counter + 204;
				if(counter>4095){
					counter=4095;
				}
				d_to_a(0, counter);
				shownum(counter);
				putchar(0x0A);
				putchar(0x0D);
	}
}		
