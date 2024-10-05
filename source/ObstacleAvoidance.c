#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "Utilities.h"
#include "stdbool.h"
#include "SpeedControl.h"
#include "MotorControl.h"

volatile int endCnt = 0;
volatile int startCnt = 0;
bool startFlag = false;



void initSensor()
{
	//init portA13
	SIM->SCGC5 |= (1<<12) | (1<<9);
	PORTA->PCR[13] &= ~0x700 ;
	PORTA->PCR[13] = 0x700 & (1 << 8);
	PORTD->PCR[2] &= ~0x700 ;
	PORTD->PCR[2] |= 0x700 & (1 << 8);
	//Set ports for GPIO output PTD2
	GPIOA->PDDR &= ~(1 << 13); //set to input
	GPIOD->PDDR|=(1<<2);
	//Set ports for GPIO PTA6 interrupt on rising edge
	SIM->SCGC6 |= (1<<23);
	PIT->MCR = 0x00;
	PIT->CHANNEL[0].LDVAL = 2400000; // setup timer 1 for 2400000 cycles ie 100ms
	PIT->CHANNEL[0].TCTRL &= ~(0x3);//clear TCTRL

	NVIC_EnableIRQ(30);

};
void initServo()
{
	SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK;
	//Set pta12 to be driven by tpm1
	PORTA->PCR[12] &= ~PORT_PCR_MUX_MASK ; // Clear MUX
	PORTA->PCR[12] |= 0x300; // Drive Pin with TPM1

	SIM->SCGC6 |= (1<<25); //Clk enable tpm1
	SIM->SOPT2 |= (0x2<<24); //Use OSCERCLK]
	TPM1->SC = (0x1<<5) | (0x1<<7)| 0x2;//set CPWMS to 0, reset timer overflow, set prescaler to 8
	TPM1->MOD = 4999; //set mod to 1999 for a 2ms period as 8Mhz/8*0.002 = 2000
	TPM1->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);  // Edge-aligned high-true PWM Channel 0
	TPM1->CONF |= (0x3<<6); //Enable debug mode for tpm1
	TPM1->CONTROLS[0].CnV = 1500;//2400-180 : 600-0 : 1500-90
	TPM1->SC |= (1<<7);
	TPM1->SC |= (1<<3);//start clock
};

void initObstacle()
{
	initServo();
	initSensor();
}

void moveServo(float degree)
{
	if(degree<=0) degree = 1;
	float final = degree/180; //calculate ratio
	final = final * 1800 + 625; //multiply the difference between the max and min servo time ranges to get speed in counter value
	TPM1->CONTROLS[0].CnV = (int)final;//set the servo motor to the new value
	delay_ms(800);//wait 0.5 seconds for server to move
}

int pulse()
{
	endCnt = 0;
	startCnt =0;
	GPIOD->PDOR |= (1 << 2);
	//delay_us(20);
	int i = 0;
	while(i<60){i++;};
	GPIOD->PDOR &= ~(1 << 2);//clear signal
	PIT->CHANNEL[0].TCTRL = 0x1;

	while(!(GPIOA->PDIR & (1<<13))){};
	startCnt = PIT->CHANNEL[0].CVAL;
	while(GPIOA->PDIR & (1<<13)){};
	endCnt = PIT->CHANNEL[0].CVAL;

	int pulseTime = 0;
	pulseTime = endCnt-startCnt;

	//printf("start: %d ,,,, end: %d ,,,, diff: %d\n",startCnt,endCnt,abs(pulseTime/24/48));//*340/100/2);	while(!(GPIOA->PDIR & (1<<13))){};

	return abs(pulseTime/24/48); //return the final value in CM
}


void turnLeft()
{
	initSpeedControl();
	int distLt=distL;
	int distRt=distR;
	distL = 0;
	distR = 0;
	controlMotor(1,2,5000);
	controlMotor(2,1,5100);
	while(distR < 63 && distL < 63)
	{
		if(distR >= 63)
		{
			STR();
		}
		if(distL >= 63)
		{
			STL();
		}
	}
	distL = distLt;
	distR = distRt;
	AllStop();
}
void turnLeftAmount(int amount)
{
	delay_ms(10);
	//int distLt=distL;
	int distRt=distR;
	initSpeedControl();
	distR = 0;
	controlMotor(2,1,7000);
	controlMotor(1,1,1);
	while(distR < amount){}
	//distL = distLt;
	distR = distRt;
	AllStop();
}
void turnRightAmount(int amount)
{
	delay_ms(10);
	int distLt=distL;
	//int distRt=distR;
	initSpeedControl();
	distL = 0;
	controlMotor(1,1,7000);
	controlMotor(2,1,1);
	while(distL < amount){}
	distL = distLt;
	//distR = distRt;
	AllStop();
}

void turnRight()
{
	int distLt=distL;
	int distRt=distR;
	initSpeedControl();
	distL = 0;
	distR = 0;
	controlMotor(1,1,5000);
	controlMotor(2,2,5100);
	while(distR < 70 && distL < 70)
	{
		if(distR >= 70)
		{
			STR();
		}
		if(distL >= 70)
		{
			STL();
		}
	}
	distL = distLt;
	distR = distRt;
	AllStop();
}

void uturn()
{
	initSpeedControl();
	distL = 0;
	distR = 0;
	controlMotor(1,2,5500);
	controlMotor(2,1,5500);
	while(distR < 126 && distL < 126)
	{
		if(distR >= 126)
		{
			STR();
		}
		if(distL >= 126)
		{
			STL();
		}
	}
	AllStop();
}


void sonar()
{
	int distLtemp = distL;
	int distRtemp = distR;

	//status flags for left and right
	int clearL = 1;
	int clearR = 1;
	unsigned int light_val_L = 0;
	unsigned int light_val_R = 0;
	//check if the robot sense a wall in front of it
	if(pulse() <= 10)
	{
		AllStop(); //stop motors
		turnLeftAmount(120);
		moveServo(1);//move servo to the 0 degree position
		controlMotor(1,1,4400);
		controlMotor(2,1,4500);
		while(pulse()<=15){};
		delay_s(2);
		delay_ms(1);
		AllStop();

		turnRight();
		distR=0;
		distL=0;
		AllStop();
		delay_ms(20);
		controlMotor(1,1,4000);
		controlMotor(2,1,4100);
		while(pulse()<=15){};
		delay_s(2);
		delay_ms(500);
		AllStop();
		distLtemp = distLtemp+distL;
		distRtemp = distRtemp+distR;

		turnRightAmount(60);
		AllStop();
		delay_ms(20);
		controlMotor(1,1,4000);
		controlMotor(2,1,4100);

		while(distL<1000)
		{
			ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
			while(!(ADC0->SC1[0] & 0x80)){ }
			light_val_L = ADC0->R[0]; // Resets COCO

			ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
			while(!(ADC0->SC1[0] & 0x80)){ }
			light_val_R = ADC0->R[0]; // Resets COCO
			if(distL>25)

			{
				if(light_val_R > 240)
				{
					turnLeftAmount(140);
					break;
				}
				else if(light_val_L > 240)
				{
					turnLeftAmount(140);
					break;
				}

			}
		}

		distL = distLtemp;
		distR = distRtemp;
	}

}

void TPM2_IRQHandler()
{
	TPM2->SC |= (1<<7);
}



