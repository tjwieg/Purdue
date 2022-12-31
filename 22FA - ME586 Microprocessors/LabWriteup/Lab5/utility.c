/**
  ******************************************************************************
  * @file    utility.c for ME586 Homework 5
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-10
  * @brief   Utility programs for HW 5 question 1 & 3
  ******************************************************************************
  */

extern void initcom();
extern char checkcom();
extern char getchar();
extern void showchar(char); // prototype for assembly routine
void showmsg(char* msg) {
  int i = 0;
  int stopflag = 0;
  while (stopflag != 1) {
    char outchar = *(msg+i);
    if (outchar == 0x00) {  // check for null ASCII to end
      stopflag = 1;
    } else {                // otherwise send character
      showchar(outchar);
      i = i+1;
    }
  }
}

short sci4(char base, char power10) {
  // Input number in scientific notation (maximum 10^4)
  // Output is number in decimal form
  short output = 0;
  if (power10 == 0) {
    output = base;
  } else if (power10 == 1) {
    output = base*10;
  } else if (power10 == 2) {
    output = base*100;
  } else if (power10 == 3) {
    output = base*1000;
  } else if (power10 == 4) {
    output = base*10000;
  }
  return(output);
}

short getanum() {
  // setup
  char stopflag = 0;
  char readyflag = 0x00;
  char input = 0x00;
  
  // default input
  char num[5] = {0,0,0,0,0};
  char digits = 0;
	short output = 0;
	char i = 0;
	initcom();

  // input loop
  while (stopflag != 1) {
    // wait to receive serial input
    while (readyflag != 0xFF) {
      readyflag = checkcom();
    }
    readyflag = 0x00; // reset flag for next loop

    // get input and react accordingly
    input = getchar();
    if ((input >= 0x30) && (input <= 0x39)) {   // if input is a number
      num[digits] = input - 0x30; // convert from ASCII to decimal
      digits = digits + 1;
    }
    if ((input == 0x0D) || (digits >= 4)) {     // if input is CR or 5th digit
      stopflag = 1;
    }
  }
  
  // convert num array to a single short
  
  for (i = 0; i < digits; i++) {
    output = output + sci4(num[i], digits - i);
  }
  return(output);
}
