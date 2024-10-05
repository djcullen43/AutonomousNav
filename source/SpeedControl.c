/*
 * SpeedControl.c
 *
 *  Created on: Oct 24, 2022
 *      Author: djcullen
 */

#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "Utilities.h"
#include "MotorControl.h"
#include "ObstacleAvoidance.h"
volatile int distR=0;
volatile int distL=0;
volatile int distM=687;
volatile int coverL=0;
volatile int coverR=0;
//Initializes ports 6,7,14,15 for interrupts
void initSpeedControl(void)
{
	//Enable clock gate for port A
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	//Set ports for GPIO PTA6 interrupt on rising edge
	PORTA->PCR[6] &= ~0xF0703 ;
	PORTA->PCR[6] = (1<<8) | (0b1001<<16); //set mux enable rising edge
	PORTA->PCR[6] |= (1<<24); //set mux enable rising edge
	GPIOA->PDDR &= ~(1 << 6); //set to input
	//Set ports for GPIO PTA7 interrupt on rising edge
	PORTA->PCR[7] &= ~0xF0703 ;
	PORTA->PCR[7] = (1<<8) | (0b1001<<16);//set mux enable rising edge
	GPIOA->PDDR &= ~(1 << 7);//set to input
	//Set ports for GPIO PTA14 interrupt on rising edge
	PORTA->PCR[14] &= ~0xF0703 ;
	PORTA->PCR[14] = (1<<8) | (0b1001<<16);//set mux enable rising edge
	GPIOA->PDDR &= ~(1 << 14);//set to input
	//Set ports for GPIO PTA15 interrupt on rising edge
	PORTA->PCR[15] &= ~0xF0703 ;
	PORTA->PCR[15] = (1<<8) | (0b1001<<16);//set mux enable rising edge
	GPIOA->PDDR &= ~(1 << 15);//set to input
	NVIC_EnableIRQ(30);
}

//Initialize the PIT module
//24MHz = 41.6667 ns 100ms/41.6667ns = 2400000
void initPIT(void)
{
	SIM->SCGC6 |= (1<<23);
	PIT->MCR = 0x00;
	PIT->CHANNEL[0].LDVAL = 2400000; // setup timer 1 for 2400000 cycles ie 100ms
	NVIC_EnableIRQ(22);
	PIT->CHANNEL[0].TCTRL = 0x3;//enable timer interrupts and start timer
}


volatile float timeElapsed = 0;//current time elapsed
volatile float totalErrorR=0;//error on right wheel
volatile float totalErrorL=0;//error on left wheel
volatile int lastR;//Last set power output of the right motor
volatile int lastL;//Last set power output of the right motor


//PI control function to adjust motor power output based on error between current
//and expected distance
void PIT_IRQHandler(void)
{
	timeElapsed += 0.1; //add 100ms to timeElapsed
	float expected = timeElapsed/7*687; //expected = timeElapsed/totaltime * totalcycles
	float Kp=5;
	float Ki=0.01;
	//calculate proportional error for each wheel
	float errorL = expected-distL;
	float errorR = expected-distR;
	totalErrorR += errorR;
	totalErrorL += errorL;
	//Control function to adjust power output
	float controlR = (Kp*(errorR)+Ki*totalErrorR);
	float controlL = (Kp*(errorL)+Ki*totalErrorL);
	float newspeedL =lastL+controlL;
	float newspeedR =lastR+controlR;
	//Check if expected distance traveled is less than one meter
	if(expected < 687)
	{
		controlMotor(1,1,(int)newspeedL);
		controlMotor(2,1,(int)newspeedR);
	}
	else //stop motors if 1 meter has been traversed
	{
		controlMotor(1,1,0);
		controlMotor(2,1,0);
	}

	//save new values for last power setting
	lastR = lastR+controlR;
	lastL = lastL+controlL;

	PIT->CHANNEL[0].TFLG = 1; // Reset the interrupt flag
}


//Interrupt handler to monitor distance traveled by each wheel
void PORTA_IRQHandler(void)
{
	//switch using the interrupt status flag register
	switch(PORTA->ISFR)
	{
	  case ((uint32_t)1<<6):
	  	PORTA->PCR[6] |= PORT_PCR_ISF_MASK;
	    distL+=1; //increment distance travelled by left wheel
	    coverL+=1;
	  	break;
	  	/*
	  case ((uint32_t)1<<7):
	  	PORTA->PCR[7] |= PORT_PCR_ISF_MASK;
	    distL+=1; //increment distance travelled by left wheel
	  	break;
	  case ((uint32_t)1<<14):
	  	PORTA->PCR[14] |= PORT_PCR_ISF_MASK;
	    distR+=1; //increment distance travelled by left wheel
	  	break;
	  	*/
	  case ((uint32_t)1<<15):
	    PORTA->PCR[15] |= PORT_PCR_ISF_MASK;
	    distR+=1; //increment distance travelled by right wheel
	    coverR+=1;
	  	break;
	  case ((uint32_t)1<<13):
	  			PORTA->PCR[13] |= PORT_PCR_ISF_MASK;
	  	  	  	startCnt = TPM2->CNT;

	  			if(startFlag==false)
	  			{
	  				startCnt = TPM2->CNT;
	  				startFlag=true;
	  			    //TPM2->SC |= 0x01 << 3; // Start the clock!

	  			}
	  			else
	  			{
	  				endCnt = TPM2->CNT;
	  				//TPM2->SC &= ~(0b11 << 3); // stop the clock!
	  				//printf("Range: %d\n",startCnt);
	  				//printf("Range: %d\n",endCnt);
	  				startFlag = false;
	  			}


	  			//while(GPIOA->PDIR & 0x2000){};
	  			//endCnt = TPM2->CNT;
	  			break;
	  default:
		  PORTA->ISFR |= 0xFFFFFFFF;
	}
}
