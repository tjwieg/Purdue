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

extern void digout(short); // from me586.h
void problem4() {
  short input = 0;
  initcom();
  while (1) {
    showmsg("Enter a 16-bit number in decimal format:");
    input = getanum();
    digout(input);
  }
}
