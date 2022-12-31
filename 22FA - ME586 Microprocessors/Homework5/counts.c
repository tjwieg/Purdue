/**
  ******************************************************************************
  * @file    counts.c for ME586 Homework 5
  * @author  Harsh Savla & TJ Wiegman
  * @version V1.0
  * @date    2022-10-10
  * @brief   Keycount program for HW 5 question 2
  ******************************************************************************
  */

extern void initcom();
extern char checkcom();
extern char getchar();
extern void shownum(short);
void keycount() {
  // setup
  char stopflag = 0;
  char readyflag = 0x00;
  char input = 0x00;
  short int counter = 0;    // counter is a 16 bit number
  initcom();
  showmsg("Press 's' to increment counter. Press 'd' to decrement. Press ESC to end.");

  // loop
  while (stopflag != 1) {
    // wait to receive serial input
    while (readyflag != 0xFF) {
      readyflag = checkcom();
    }
    readyflag = 0x00; // reset flag for next loop

    // get input and react accordingly
    input = getchar();
    if (input == 0x1B) {            // if receive 'ESC'
      stopflag = 1;
    } else if (input == 0x73) {     // if receive 's'
      counter = counter + 1;
      shownum(counter);
    } else if (input == 0x64) {     // if receive 'd'
      counter = counter - 1;
      shownum(counter);
    }
  }
}
