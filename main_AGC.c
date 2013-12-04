///////////////////////////////////////////////////////////////////////////////////////
//NAME:     main.c (DPSK receiver structures)
//DATE:     31/01/2011
//PURPOSE:  EEE8091 Implementation of wireless receiver algorithms
//Writer:  Group 1
//Third versions
////////////////////////////////////////////////////////////////////////////////////////
#include "setup.h"
#include <filters.h>

#define N_bp 151  	//bandpass filter tape
#define N_mf 20 	//Number of delay line taps

// Define global variables here

// Data buffer
char data[72] = "abcdefghijklmnopqrstuvwxyz0123456789";	

// Variable declarations
float x0;//input
float x1;//output of bandpass
float x3I;//
float x3Q;//
float x4I;//output of match filter
float x4Q;//
float dm x5I[N_mf];//buffer
float dm x5Q[N_mf];//buffer
float Ee;//energy for early part of buffer
float El;//energy for late part of buffer
float x6I;//output of symbol sync
float x6Q;//output of symbol sync

//work for AGC
float AGC[N_mf];//buffer for Automatic gain control
float xe;//output of AGC
float Ea=1.0;//average energy
float Eaim=1.3;//Aim energy

//work for costas loop
float yI=0.0;
float yQ=0.0;
float wI=1.0;
float wQ=0.0;
float eI=0.0;
float eQ=0.0;
float mu=0.001;
float d=0.0;
float d_prev=0.0;
int x7;

//work for shifting
unsigned int Nc0=1;
unsigned int Nc1=N_mf;
signed int Nc2=0;


float dm state_bp[N_bp+1];//work for bandpass filter
float pm h_bp[N_bp]=
{
	#include "bp.h"
};

float pm x2I[10]=
{
	#include "x2I.h"
};
float pm x2Q[10]=
{
	#include "x2Q.h"
};
float dm state_mf_I[N_mf+1];
float pm coeffs_mf_I[N_mf] =
{
	#include "mf_I.h"
};
float dm state_mf_Q[N_mf+1];
float pm coeffs_mf_Q[N_mf] =
{
	#include "mf_Q.h"
};

// Input scaling factor: needs to be adjusted for each board based on signal strength
float scale_in=4.0*5.0/4.294967296E9;
float scale_out=4.294967296E9/(2.5*5.0);

// Probe buffers
float dm scope1[NUM_SAMPLES/2];
float dm scope2[NUM_SAMPLES/2];

void main(void)
{	
	// Local variables
    int i,k,h=0;
	unsigned char preamble=0;
	unsigned char datachar=0;
    unsigned char count_bits=0;
    unsigned char count_bytes=0;
	unsigned char trig=0;
	unsigned char led_cnt = 0;
	
    // Setup 21364 board    
	Setup21364();	

	
	//bandpass
	for (k=0;k < N_bp+1; k++)
		{
		state_bp[k] = 0.0;
		}
		
    //Initialise buffer contents
	for (k=0;k < N_mf; k++)
		{
			x5I[k]=0.0;
			x5Q[k]=0.0;
			AGC[k]=0.0;
		}

		
		
	// Infinite loop
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
     		k=0;
			
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
    			
				//AGC part
    			xe=Eaim/Ea*AGC[19];			       	
				for(k=N_mf-1;k>0;k--)          
				{
					AGC[k]=AGC[k-1];		
				}
				AGC[0]=x1;
				for (k=0;k<20;k++)
					{
						Ea= Ea + AGC[k]*AGC[k];
					}
				Ea=Ea/20;
				
				
    			x3I=xe*x2I[h];
    			x3Q=xe*x2Q[h];
    			h++;
    			if(h>9)
    				h=0;
    			
    			x4I=fir(x3I, coeffs_mf_I, state_mf_I, N_mf);
    			x4Q=fir(x3Q, coeffs_mf_Q, state_mf_Q, N_mf);
    			
    			//x5 input by shifting
				for(k=0;k<N_mf-1;k++)
				{
					x5I[k]=x5I[k+1];
					x5Q[k]=x5Q[k+1];          
				}
				x5I[N_mf-1]=x4I;  
    			x5Q[N_mf-1]=x4Q; 

    							
    			
    			Nc0=Nc0+1; //define Nc0
		   		//shifting control
				if(Nc0 == Nc1)
				{
					Ee=0;
					El=0;
					for (k=0;k<10;k++)
					{
						Ee = Ee + x5I[k]*x5I[k] + x5Q[k]*x5Q[k];
						El = El + x5I[k+10]*x5I[k+10] + x5Q[k+10]*x5Q[k+10];						
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

			        
			        x6I=0.5*(x5I[9]+x5I[10]);		
    				x6Q=0.5*(x5Q[9]+x5Q[10]);
    				

      				//Costas loop
    				yI=wI*x6I-wQ*x6Q;
    				yQ=wQ*x6I+x6Q*wI;
    				d=(yI>0)?1.0:-1.0;
    				eI=d-yI;
    				eQ=-yQ;
    				wI=wI+mu*((d-wI*x6I+wQ*x6Q)*x6I+(-wQ*x6I-wI*x6Q)*x6Q);
    				wQ=wQ+mu*((-wQ*x6I-wI*x6Q)*x6I-(d-wI*x6I+wQ*x6Q)*x6Q);
    				
    				if(d == d_prev)
    				{
    					x7=0;
    				}
    				else x7=1;
   					
    				d_prev=d;
    				
					//Frame synchronisation Search ++++
					if(trig == 0)
					{
						preamble=preamble>>1;
						if(x7==1)
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
						if(x7==1)
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
    			scope1[k]=x3I;
    			scope2[k++]=x3Q;
    			
    			/*
    				Important Notes: the code below implements:
    			    	1. 32-to-24 bit convesion and output to DAC 1 & 2
     		 	    	2. Index 2*i+1 corresponds to DAC channel 1, 2i to DAC channel 2
     		 	    	3. Full-Scale Output Voltage at Each Pin (Single-Ended) 1 Vrms or 2.8Vpp.
     		 	           Current setup: DACVOL_MAX in init1835viaSPI.c gives ~ 2.8Vpp
				4. Depending on the input signal strength additional scaling is required.
     		 		5. The code below sends x0 and -x0 to the DAC outputs.
				*/
				sample_out[2*i+1]=(unsigned int)(((int)(yI*scale_out))>>8);	// DAC 1				
     		 	sample_out[2*i]=(unsigned int)(((int)(yQ*scale_out))>>8);	// DAC 2
     		}
     		     	  	 
        }

    }
    
}
