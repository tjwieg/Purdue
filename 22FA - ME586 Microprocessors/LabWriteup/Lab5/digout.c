/**
  ******************************************************************************
  * @file    digout.c for ME586 Homework 5
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-10
  * @brief   Main program body for HW 5 question 4
  ******************************************************************************
  */

#include "ME586.h"

void problem4() {
	short input;
	//char output;
  initcom();
	initports(0xF00);
  while (1) {
    showmsg("Enter a 16-bit number in decimal format:");
		putchar(0x0D);
		putchar(0x0A);
    input = getanum();
		putchar(0x0D);
		putchar(0x0A);
    digout(input);
  }
}
