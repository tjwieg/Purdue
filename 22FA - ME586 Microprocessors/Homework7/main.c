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
int* ram_ptr;       // address of storage start
int* adc_ptr;       // address of last stored value
int ram_allocated;  // amount of RAM that has actually been allocated
int ram_size = 0;
float time_period = 100;  // in milliseconds
float Kp = 1;
float Ki = 0;
float Kd = 0;
float setpoint = 0;
float error_prior = 0;
float integral = 0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


int main(void) {
    // Setup
    char input_choice;
    char input_mode = 1;
    int stored_actual = 0;
    disable_timer_interrupt();  // start without interrupts
    initcom();  // initialize serial
    initadc();  // initialize analog in
    initdac();  // initialize analog out
    
    // User input
    while (input_mode == 1) {
        printf("\nChange parameters?\n");
        printf(" [S] Provide a setpoint\n");
        printf(" [G] Input new gains\n");
        printf(" [T] Set the sampling period\n");
        printf(" [P] Change RAM storage size\n");
        printf(" [R] Run the controller\n");
        printf(" [D] Output stored values\n");
        printf(" [X] Exit\n");
        printf("Input: ");
        WaitForKeypress();
        input_choice = getchar();
        printf("\n\n")

        switch (input_choice) {            
            case 'S' : // provide a setpoint
                printf("Setpoint: ");
                setpoint = getfloat();
                printf("\n");
            break;

            case 'G' :  // change gains
                printf("Kp: ");
                Kp = getfloat();
                printf("\nKi: ");
                Ki = getfloat();
                printf("\nKd: ");
                Kd = getfloat();
                printf("\n");
            break;
            
            case 'T' :  // timer period
                printf("Time period (ms): ");
                time_period = getfloat();
                printf("\n");
            break;

            case 'P' : // change RAM storage size
                printf("Number of values to store: ");
                ram_size = getnum();
                printf("\n");
            break;
            
            case 'R' : // run the controller
                // Allocate RAM storage size
                if (ram_ptr == NULL) {
                    ram_ptr = (int*) calloc(ram_size * sizeof(int));
                } else {
                    free(ram_ptr);
                    ram_ptr = (int*) calloc(ram_size * sizeof(int));
                }
                ram_allocated = ram_size;
                adc_ptr = ram_ptr;
                
                // Report allocated RAM to user
                if (ram_size == 0) {
                    printf("Not recording measured values.\n")
                } else if (ram_ptr == NULL) {
                    printf("Error! Failed to allocate memory for measured values.\n")
                } else {
                    printf("Recording first ");
                    shownum(ram_allocated);
                    printf(" measured values.\n");
                }
                
                // Start running controller
                printf("Running...\n");
                inittime(time_period);
                restore_timer_interrupt();
            break;

            case 'D' : // output data from RAM
                disable_timer_interrupt(); // don't want data to change mid-readout
                stored_actual = (adc_ptr - ram_ptr) / sizeof(int);
                printf("First ");
                shownum(stored_actual);
                printf(" measured values:\n");
                for (int* out_ptr = ram_ptr; i <= adc_ptr; out_ptr += sizeof(int)) {
                    // start at ram_ptr, count up to adc_ptr, in increments of sizeof(int)
                    shownum(*out_ptr);
                    printf("\n"); // each number gets a new line
                }
                restore_timer_interrupt();
            break;
            
            case 'X' : // exit
                input_mode = 0;
                disable_timer_interrupt();
                printf("Quitting...\n");
            break;

            default:
                printf("Input not recognized.\n")
            break;
        }
    }
    while (1) {};
} //end of main program


void timehand(void) {
    // Setup
    int measured_ADC;
    float error;
    float derivative;
    float output;

    // PID Controller
    measured_ADC = a_to_d(0);
    error = setpoint - measured_ADC;
    integral = integral + (error * time_period);
    derivative = (error - error_prior) / time_period;
    output = (Kp * error) + (Ki * integral) + (Kd * derivative);

    if (output > OUTPUT_MAX) output = OUTPUT_MAX;
    if (output < OUTPUT_MIN) output = OUTPUT_MIN;
    error_prior = error;
    d_to_a(0, output);

    // Store PID values
    if (adc_ptr < (ram_ptr + (ram_allocated * sizeof(int))) {
        *adc_ptr = measured_ADC;
        adc_ptr = adc_ptr + sizeof(int);
    }
}

void inthand(void){
}

/***********************END OF FILE****/
