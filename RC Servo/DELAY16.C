/*
 *	Delay functions fro 16F877 running at 16 Mhz
 *	written by Jim Sedgwick 5/9/2006
 *	See DELAY16MHZ.H for details
 *
 *	Make sure this code is compiled with full optimization!!!
 */

#include "DELAY16.H"
#include <pic18.h>


// Millisecond delay routine for 16 Mhz PICs
// Accepts any unsigned integer value
// Will create a delay in milliseconds
// for that value.
void DelayMs(unsigned int i)
{
int j;
unsigned char k;

	for(j=0; j<i; j++)
	{
		k=255;
		while(k--);
		k=255;
		while(k--);
		k=255;
		while(k--);
		k=255;
		while(k--);
	}
}


// Microsecond delay routine for 16 Mhz PICs
// Accepts any value from 1 to 255 microseconds
void DelayUs(unsigned char i)
{
	while(i--);
}


 
