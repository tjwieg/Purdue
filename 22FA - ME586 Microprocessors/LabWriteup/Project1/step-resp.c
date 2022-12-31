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
#define degree_start 2048
#define degree_end	 2176
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
int array[1000];
int counter=0;
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


int main(void) {
	initcom();
	initdac();
	initadc();
	
	d_to_a(0, degree_start);				// set to 0 degrees
	WaitForKeypress();
	inittime(10);
	while (counter != 1000) {};
	disable_timer_interrupt();
	
	for (int i = 0; i < 1000; i++) {
		printf("\n\r");
		shownum(array[i]);
	}
} //end of main program


void timehand(void){
	if(counter<200){
		d_to_a(0, degree_start);	// set to 0 degrees
	} else { 
		d_to_a(0, degree_end);		// set to end position
	}
	if (counter < 1000) {
		array[counter]=a_to_d(1);
		counter = counter + 1;
	}
}

void inthand(void){
}

/***********************END OF FILE****/
