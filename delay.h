/*
 *	Delay functions for HI-TECH C on the PIC18
 *
 *	Functions available:
 *		DelayUs(x)	Delay specified number of microseconds
 *		DelayMs(x)	Delay specified number of milliseconds
 *
 *	Note that there are range limits: 
 *	- on small values of x (i.e. x<10), the delay becomes less
 *	accurate. DelayUs is accurate with xtal frequencies in the
 * 	range of 4-16MHZ, where x must not exceed 255. 
 *	For xtal frequencies > 16MHz the valid range for DelayUs
 *	is even smaller - hence affecting DelayMs.
 *	To use DelayUs it is only necessary to include this file.
 *	To use DelayMs you must include delay.c in your project.
 *
 *	Set the crystal frequency in the CPP predefined symbols list
 *	on the PICC-18 commmand line, e.g.
 *	picc18 -D_XTAL_FREQ=4MHZ
 *
 *	or
 *	picc18 -D_XTAL_FREQ=100KHZ
 *	
 *	Note that this is the crystal frequency, the CPU clock is
 *	divided by 4.
 *
 *	MAKE SURE this code is compiled with full optimization!!!
*/

#define	MHZ	*1000000

#ifdef XTAL_FREQ
 // if detect old symbol, convert to new symbol and warn about name change
 #define _XTAL_FREQ XTAL_FREQ
 #warning Preprocessor symbol XTAL_FREQ has been deprecated. Now used _XTAL_FREQ.
#endif

#ifndef	_XTAL_FREQ
#define	_XTAL_FREQ	4MHZ		/* Crystal frequency in MHz */
#endif

#if	_XTAL_FREQ < 8MHZ
#define	uS_CNT 	238			/* 4x to make 1 mSec */
#endif

#if	_XTAL_FREQ == 8MHZ
#define uS_CNT  244
#endif

#if	_XTAL_FREQ > 8MHZ
#define uS_CNT  246
#endif

#define FREQ_MULT	(_XTAL_FREQ)/(4MHZ)

#define	DelayUs(x)	{ unsigned char _dcnt; \
			  if(x>=4) \
				_dcnt=(x*(FREQ_MULT)/5); \
			  else \
				_dcnt=1; \
			  while(--_dcnt > 0){ \
					asm("bra $+1");\
					continue;\
			  }\
			} 

// This routine is now deprecated - now #include htc.h and use __delay_ms() instead.
extern void DelayMs(unsigned char);


