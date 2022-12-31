/**
  ******************************************************************************
  * @file    main.c 
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.1.0
  * @date    2022-10-24
  * @brief   Main program body for ME586 Homework 7
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "ME586.h"
#include <stdlib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define OUTPUT_MAX 4095
#define OUTPUT_MIN 0
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int ram_ptr[1000];        // measured data stored in memory
int stored_actual = 0;    // number of values actually written
float time_period = 10;   // in milliseconds
float Kp = -0.14411;
float Ki = -0.393539;
float Kd = 0.00;
float setpoint = 0;
float error_prior = 0;
float integral = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


int main(void) {
    // Setup
    char input_choice;
    char input_mode = 1;
    initcom();  // initialize serial
    initadc();  // initialize analog in
    initdac();  // initialize analog out
    
    // User input
    while (input_mode == 1) {
        printf("\n\rChange parameters?\n\r");
        printf(" [S] Provide a setpoint\n\r");
        printf(" [G] Input new gains\n\r");
        printf(" [T] Set the sampling period\n\r");
        printf(" [R] Run the controller\n\r");
        printf(" [D] Output stored values\n\r");
        printf(" [X] Exit\n\r");
        printf("Input: ");
        input_choice = getchar();
        printf("\n\r\n\r");

        switch (input_choice) {            
            case 'S' : // provide a setpoint
                printf("Setpoint (RPM): ");
                setpoint = getfloat();
                printf("\n\r");
            break;

            case 'G' :  // change gains
                printf("Kp: ");
                Kp = getfloat();
                printf("\n\rKi: ");
                Ki = getfloat();
                printf("\n\rKd: ");
                Kd = getfloat();
                printf("\n\r");
            break;
            
            case 'T' :  // timer period
                printf("Time period (ms): ");
                time_period = getfloat();
                printf("\n\r");
            break;
            
            case 'R' : // run the controller
                printf("Running...\n\r");
                inittime(time_period);
            break;

            case 'D' : // output data from RAM
                disable_timer_interrupt(); // don't want data to change mid-readout
                printf("First ");
                shownum(stored_actual);
                printf(" measured values:\n\r");
                for (int i = 0; i <= stored_actual; i++) {
                    shownum(ram_ptr[i]);
                    printf("\n\r"); // each number gets a new line
                }
                restore_timer_interrupt();
            break;
            
            case 'X' : // exit
                input_mode = 0;
                disable_timer_interrupt();
                d_to_a(0, 2048);
                printf("Quitting...\n\r");
            break;

            default:
                printf("Input not recognized.\n\r");
            break;
        }
    }
    while (1) {};
} //end of main program


void timehand(void) {
    // Setup
    int measured_ADC;
	float measured_engvolt;
    float error;
    float derivative;
    float output;
	float reference;

    // Get position from ADC input
    measured_ADC = a_to_d(1);

    // Convert ADC count to engine voltage
    measured_engvolt = 0.0024414*measured_ADC - 5;

    // Convert ADC count to RPM and save to memory
    if (stored_actual < 1000) {
        ram_ptr[stored_actual] = -0.6984*measured_ADC + 1504;   
        stored_actual = stored_actual + 1;
    }
		
    // PID Controller
	reference = -0.0033*setpoint + 0.0762; // convert setpoint from RPM to engine voltage
    error = reference - measured_engvolt;
    integral = integral + (error * (time_period / 1000)); // T/1000 to convert ms to seconds
    derivative = (error - error_prior) / (time_period / 1000);
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    error_prior = error;

    // Convert output voltage to DAC count, bounded from 0 to 4095
    output = 204.8*output + 2048;
	if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;
	
    d_to_a(0, output);
}

void inthand(void){
}

/***********************END OF FILE****/
