#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "MotorControl.h"
#include "SpeedControl.h"
#include "Utilities.h"
#include "ObstacleAvoidance.h"

void initADC()
{
	unsigned short cal_v = 0;
	unsigned char light_val = 0;
    //PTE16 left PTE17 Right
	SIM->SCGC6 |= (1<<27);//enable clock for adc0
	SIM->SCGC5 |= (1<<13);//enable portE clock

	// Setup Analog Input - Default is analog (PTE22), No need to do anything.

	// Setup ADC Clock ( < 4 MHz)
	ADC0->CFG1 = 0;  // Default everything.

	// Analog Calibrate
	ADC0->SC3 = 0x07; // Enable Maximum Hardware Averaging
	ADC0->SC3 |= 0x80; // Start Calibration
	// Wait for Calibration to Complete (either COCO or CALF)
	while(!(ADC0->SC1[0] & 0x80)){}
	// Calibration Complete, write calibration registers.
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 +ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;
	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 +
	ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;
	ADC0->SC3 = 0; // Turn off Hardware Averaging

	while(1)
	{
		ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		//delay(1000);
		light_val = ADC0->R[0]; // Resets COCO
		printf("Light_val = %d",light_val);
	}
}

void autoDrive()
{
	while(1)
	{
		//status flags for left and right
		int clearL = 1;
		int clearR = 1;

		//check if the robot sense a wall in front of it
		if(pulse() <= 15)
		{
			AllStop(); //stop motors
			moveServo(1);//move servo to the 0 degree position
			if(pulse() <= 25)
				clearR = 0; //set flag to false when wall is detected left
			moveServo(180);//move servo to the 180 degree position
			if(pulse() <= 25)
				clearL = 0; //set flag to false when wall is detected right
			moveServo(90);//move servo to the 90 degree position
			//printf("ClearL: %x\n",clearL);
			//printf("ClearR: %x\n",clearR);

			turnLeft(); //turn left
			controlMotor(1,2,5500);
			controlMotor(2,1,5600);
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
			turnRight();

			clearL = 1; //reset the status flags
			clearR = 1;
		}
	}

}

void initI2C() {

    // Enable Clock Gating for I2C module and Port
	SIM->SCGC4 |= (1<<7);
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
    // Setup Pin Mode for I2C
	PORTC->PCR[8] = 0x7 & (1 << 9); //configure pcr for I2C SCL
	PORTC->PCR[9] = 0x700 & (1 << 9);//configure pcr for I2C SD
	// Write 0 to all I2C registers
	I2C0->A1 = 0;
	I2C0->F = 0;
    // Write 0x50 to FLT register (clears all bits)
	I2C0->FLT = 0x50;
    // Clear status flags

    // Set I2C Divider Register (Choose a clock frequency less than 100 KHz)
	I2C0->F |=0;
    // Set bit to enable I2C Module

}



