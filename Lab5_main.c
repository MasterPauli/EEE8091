///////////////////////////////////////////////////////////////////////////////////////
//NAME:     main.c (DPSK receiver structures)
//DATE:     12/11/2013
//PURPOSE:  EEE8091 Implementation of wireless receiver algorithms
//Writter:  Group 1
//First versions
////////////////////////////////////////////////////////////////////////////////////////
#include "setup.h"
#include <filters.h>

#define N_mf 20 	//Number of delay line taps
#define N_bp 101  	//bandpass filter tape

//date buffer
char date[72] = "abcdefghijklmnopqrstuvwxyz0123456789";

// Variable declarations
float x0;//input
float x1;//output of bandpass
float x3;//output of delay-line
float x4;//output of mached-filter output
float Ee;//energy for early part of buffer
float El;//energy for late part of buffer
char data[72];

float dm x2[N_mf];//delay line array
float dm x5[N_mf];//buffer
unsigned int Nc0=1;
unsigned int Nc1=N_mf;
signed int Nc2=0;
float Em;
float dm state_bp[N_bp+1];//work for bandpass filter
float pm h_bp[N_bp]=
{
	#include "bp1.h"
};

//declare the matched filter
float dm state_mf[N_mf+1];
float pm h_mf[N_mf] =
{
	#include "mf.h"
};


// Input scaling factor: needs to be adjusted for each board based on signal strength
float scale_in=10.0*5.0/4.294967296E9;
float scale_out=4.294967296E9/(2.5*5.0);

// Probe buffers
float dm scope1[NUM_SAMPLES/2];
float dm scope2[NUM_SAMPLES/2];




void main(void)
{	
	// Local variables
    int i,k;
	unsigned char preamble=0;
	unsigned char datachar=0;
    unsigned char count_bits=0;
    unsigned char count_bytes=0;
	unsigned char trig=0;
	unsigned char led_cnt = 0;
	
	
    // Setup 21364 board    
	Setup21364();	
	
	for (k=0;k < N_bp+1; k++)
		{
		state_bp[k] = 0.0;
		}
	
	for (k=0; k< N_mf ; k++)
		{
			x2[k]=0.0;
		}

	for (k=0;k < N_mf+1; k++)
		{
			state_mf[k]=0.0;
		}
		
//Initialise buffer contents
	for (k=0;k < N_mf; k++)
		{
			x5[k]=0.0;
		}
		
		
    for(;;)
    {
     	while(blockReady)
     	{
     		//Clear the Block Ready Semaphore
    		blockReady = 0;
    
    		//Set the Processing Active Semaphore before starting processing
    		isProcessing = 1;
					
    		// Pointer to input samples
    		unsigned int * sample_in=src_pointer[int_cntr];
     		
    		// Pointer to output ssamples
    		unsigned int * sample_out=src_pointer[int_cntr];
    		
			
			for(i=0;i<NUM_SAMPLES/2;i++)
     		{      
     		     			
     		 	/*	
     		 	 	Important Notes:
     		 		1. Conversion from 24 to 32 bit and input scaling  
     		 		2. Index [2*i+1] corresponds to ADC channel 1, [2*i] to ADC channel 2
     		 		3. The differential input range for the ADC is ~2.8Vpp
     		 		4. Do not overdrive the ADC with the signal generator (see point 3 for range)
     		 		5. C-language elements implemented: bit-shifting, pointer casting, array indexing   
     		 	*/	   		 		
    			x0=(float)((int)(sample_in[2*i+1]<<8)*scale_in);
    			x1=fir(x0,h_bp,state_bp,N_bp);
    			
				//demodulator
    			x3=x1*x2[19];			       	
				for(k=N_mf-1;k>0;k--)          
				{
					x2[k]=x2[k-1];		
				}
				x2[0]=x1;

				//low pass filter
				x4=fir(x3, h_mf, state_mf, N_mf);
				

				
				//x5 input by shifting
				for(k=0;k<N_mf-1;k++)
				{
					x5[k]=x5[k+1];        
				}
				x5[N_mf-1]=x4;    
		   
		   
				Nc0=Nc0+1; //define Nc0
		   
				if(Nc0 == Nc1)
				{
					Ee=0;
					El=0;
					for (k=0;k<10;k++)
					{
						Ee= Ee + x5[k]*x5[k];
						El = El + x5[k+10]*x5[k+10];
					}
			


										//set nc2 to vary nc1
			        if(Ee > El)
			        Nc2=Nc2+1;
	    		    else if(Ee < El)
					Nc2=Nc2-1;
		            else
            		Nc2=Nc2; 
			
					//set nc1 to control shifting of X5
            		if(Nc2 == 10) 
            		{
	          			Nc2=0;
	          			Nc1=19;
	        		}	
	        		else if (Nc2 == -10)
			        {
			          Nc2=0;
			          Nc1=21;
		        	}
			        else
			        	Nc1=20;


	        
			        Nc0=0; //reset the counter for X5shifting
			        
			        
	        		Em=0.5*(x5[9]+x5[10]);			
					//Frame synchronisation Search ++++
					if(trig == 0)
					{
						preamble=preamble>>1;
						if(Em<0)
						{
							preamble=preamble+0x80000000;
						}
				
						if(preamble == 0x2B2B2B2B)
						{
							trig=1;
							preamble=0;
							led_cnt=led_cnt%2;
							LED_ON(led_cnt++);
						}
					}
					//Frame synchronisation get info
					else     // data detection should happened in next bits!!!
					{
						datachar>>=1;
						if(Em<0)
						{
							datachar+=0x80;
						}
						count_bits++;

					
						if(count_bits == 8)
						{
							count_bits=0;
							data[count_bytes] = datachar;
							count_bytes++;
							datachar=0;
							if(count_bytes>71)
							{
								count_bytes = 0;
								trig=0;
							}
						}

						
					}
	     	
	     	
	     	
	     	
	       		}
	 
    			// Buffer ADC 1 output for inspection after DSP is halted
    			scope1[k]=x0;
    			scope2[k++]=x0;
    			
    			/*
    				Important Notes: the code below implements:
    			    1. 32-to-24 bit convesion and output to DAC 1 & 2
     		 	    2. Index 2*i+1 corresponds to DAC channel 1, 2i to DAC channel 2
     		 	   	3. Full-Scale Output Voltage at Each Pin (Single-Ended) 1 Vrms or 2.8Vpp.
     		 	       Current setup: DACVOL_MAX in init1835viaSPI.c gives ~ 2.8Vpp
			       	4. Depending on the input signal strength additional scaling is required.
     		 	   	5. The code below sends x0 and -x0 to the DAC outputs.
				*/
				sample_out[2*i+1]=(unsigned int)(((int)(Em*scale_out))>>8);	// DAC 1				
     			sample_out[2*i]=(unsigned int)(((int)(x4*scale_out))>>8);	// DAC 2
     		}
     		     	  	 
        }

    }
    
}
