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
        // Start counting from zero
        counter = 0;
        while (nonzero == 1) {
            // Output current counter value and then check comparator
            digout(counter);
            comparator = digin();
            
            // If comparator has been triggered, then stop counting
            if (comparator < 0x100) {
                nonzero = 0;
            
            // Otherwise increase the counter and try again
            } else if (counter < 0xFF) {
                counter = counter + 1;
            
            // If counter reaches 0xFF, then stop counting
            } else {
                nonzero = 0;
            }
        }
        shownum(counter);
		putchar(0x0A);
		putchar(0x0D);
        nonzero = 1;
    }
}
