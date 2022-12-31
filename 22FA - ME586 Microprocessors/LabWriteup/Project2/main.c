/**
  ******************************************************************************
  * @file    main.c 
  * @author  Zhou Zeng
  * @version V1.1.0
  * @date    10-Oct-2018
  * @brief   Main program body.
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "ME586.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define EXTI_PR 0x40010414
#define ZEROOUT 1.6 // determined empirically
#define OUTPUT_MAX 10.0
#define OUTPUT_MIN 0.0
#define samplerate 10 // milliseconds
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int ram_ptr[1000];        // measured data stored in memory
int stored_actual = 0;    // number of values actually written
int start_record = 0;
float Kp = 0.01;
float Ki = 0.01;
float Kd = 0;
float error_prior = 0;
float integral = 0;
int clicks = 0;
int outclicks = 0;
int desclicks = 0;
int *p = (int *) EXTI_PR;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

int main(void) {
	initcom();
	initint();
	initdac();
	inittime(samplerate);
	
	printf("Desired RPM? ");
	desclicks = getnum();
	printf("\n\r");
	start_record = 1;
	
	while(stored_actual < 999) {}
	
	disable_timer_interrupt();
	d_to_a(0, 2048);
	printf("First ");
  	shownum(stored_actual);
  	printf(" measured values:\n\r");
  	for (int i = 0; i <= stored_actual; i++) {
  		shownum(ram_ptr[i]);
  		printf("\n\r"); // each number gets a new line
  	}
} //end of main program


void timehand(void){
	// Setup
  	float error;
  	float derivative;
  	float output;
	float reference = 0;
	
	// Wait to change until 2 seconds
	if (stored_actual > 200) {
		reference = desclicks;
	}
	
	// Convert to RPM and reset clicks
	outclicks = clicks*(1000/samplerate);
	clicks = 0;
	
	// Save RPM to memory
	if (stored_actual < 1000) {
    if (start_record == 1) {
			ram_ptr[stored_actual] = outclicks;
			stored_actual = stored_actual + 1;
		}
  	}
	
	// PID Control
	error = reference - outclicks;
  	integral = integral + (Ki * error * (samplerate / 1000)); // convert ms to seconds
  	derivative = (error - error_prior) / (samplerate / 1000);
	error_prior = error; // save for next D term
  	output = (Kp * error) + integral + (Kd * derivative) + ZEROOUT; // in volts
	
	// Bound output
	if (output > OUTPUT_MAX) output = OUTPUT_MAX;
  	if (output < OUTPUT_MIN) output = OUTPUT_MIN;
	
	// Convert to counts and output via DAC
	output = output*(2048/10) + 2048;
	d_to_a(0, output);
}

void inthand(void){
	*p = 2; // reset pending register
	clicks=clicks+1;
}

/***********************END OF FILE****/
