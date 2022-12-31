/* ME586.h */

#ifndef __ME586
#define __ME586

#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)

#include "stm32f10x_exti.h"
#include "stdio.h"
/*function prototypes*/
void timehand(void);
void inthand(void);
void initcom (void);
void initdac(void);
void initadc(void);
void initports(int direction);
void inittime(float sample_time);
void initint(void);
void initPWM(void);
void outputPWM(int value);
void disable_timer_interrupt(void);
void restore_timer_interrupt(void);
void d_to_a(int channel, int value);
int a_to_d(int channel);
void digout(short data);
short digin(void);
unsigned char checkcom(void);
float getfloat(void);
void WaitForKeypress(void);
void shownum(int value);
void initEncoder(void);

#ifdef USE_ABSOLUTE_ENCODER
void Encoder1A(void);
void Encoder1B(void);
void Encoder2A(void);
void Encoder2B(void);
void resetEncoder1Pos(void);
void resetEncoder2Pos(void);
void teachEncoder1Pos(int);
void teachEncoder2Pos(int);
int getEncoder1Pos(void);
int getEncoder2Pos(void);
#endif

#ifdef USE_INCREMENTAL_ENCODER
void Encoder1(void);
void Encoder2(void);
void resetEncoder1Count(void);
void resetEncoder2Count(void);
int getEncoder1Count(void);
int getEncoder2Count(void);
#endif

#define CASSERT(predicate, file) _impl_CASSERT_LINE(predicate,__LINE__,file)
#define _impl_PASTE(a,b) a##b
#define _impl_CASSERT_LINE(predicate, line, file) \
    typedef char _impl_PASTE(error_##file##_,line)[2*!!(predicate)-1];

#endif
