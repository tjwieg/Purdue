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

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define OUTPUT_MAX 10
#define OUTPUT_MIN -10
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int ram_ptr[1000];        // measured data stored in memory
int stored_actual = 0;    // number of values actually written
float time_period = 10;   // in milliseconds
float Kp = 0.42496;
float Ki = 111.71;				// K values from system ID calculations
float Kd = 0.93048;
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
                printf("Setpoint (degrees): ");
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
		float measured_angle;
    float error;
    float derivative;
    float output;

    // Get position from ADC input
    measured_ADC = a_to_d(1);
    measured_angle = 0.1729*measured_ADC - 535.34; // convert ADC count to degrees
    if (stored_actual < 1000) {
        ram_ptr[stored_actual] = measured_angle;   // store degrees to memory
        stored_actual = stored_actual + 1;
    }
    
    // PID Controller
    error = setpoint - measured_angle;
    error = 0.0024*error; 	// convert degrees to ADC voltage (no offset inside PID)
    integral = integral + (error * (time_period / 1000)); // convert ms to seconds
    derivative = (error - error_prior) / (time_period / 1000);
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;
    error_prior = error;
    output = output*205.52 + 2049.8; // convert voltage to DAC
    d_to_a(0, output);
}

void inthand(void){
}

/***********************END OF FILE****/
