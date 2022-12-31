/**
  ******************************************************************************
  * @file    counter_ADC.c for ME586 Homework 6
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-17
  * @brief   Software-driven ADC based on an external DAC and comparator
  ******************************************************************************
  */

#include "ME586.h"

extern void WaitForKeypress();
extern void initports(int);
extern short digin();
extern void digout(short);
extern void shownum(int);
void counter_ADC() {
    short counter;
    short comparator;
    char nonzero = 1;

    // expect serial communications to be initialized already
    WaitForKeypress();
    initports(0xF00); // PB5-12 are outputs, PC6-9 are an inputs

    while (1) {
        counter = 0;
        while (nonzero == 1) {
            digout(counter);
            comparator = digin();
            if (comparator < 0x100) {     // if 9th bit of 'comparator' is zero
                nonzero = 0;
            } else if (counter < 0xFF) {  // counter has maximum value of 1 byte
                counter = counter + 1;
            } else {
                counter = 0;
            }
        }
        shownum(counter);
        nonzero = 1;
    }
}
