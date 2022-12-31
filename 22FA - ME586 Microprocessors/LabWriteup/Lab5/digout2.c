/**
  ******************************************************************************
  * @file    digout2.c for ME586 Lab 5
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-11
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
    printf("Enter a 16-bit number in decimal format:");
		putchar(0x0D);
		putchar(0x0A);
    input = getnum();
		putchar(0x0D);
		putchar(0x0A);
    digout(input);
  }
}
