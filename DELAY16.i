#line 1 "DELAY16.C"
#line 1 "DELAY16.C"

#line 7 "DELAY16.C"
 

#line 1 "./DELAY16.H"

#line 9 "./DELAY16.H"
 



extern void DelayUs(unsigned char);	
extern void DelayMs(unsigned int);	

#line 9 "DELAY16.C"








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




void DelayUs(unsigned char i)
{
	while(i--);
}


 
