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

extern void initadc();
extern void initdac();
extern int a_to_d(int);
extern void d_to_a(int, int);
void analog_copy() {
    // Setup
    int input;
    initadc();
    initdac();

    // Read and write analog signals
    while (1) {
        input = a_to_d(1);
        d_to_a(0, input);
    }
}
