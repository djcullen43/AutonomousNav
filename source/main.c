/*
 * Copyright 2016-2022 NXP
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
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
	cal_v = ADC0->CLP0 + ADC0->CLP1 + ADC0->CLP2 + ADC0->CLP3 + ADC0->CLP4 +
	ADC0->CLPS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->PG = cal_v;
	cal_v = 0;
	cal_v = ADC0->CLM0 + ADC0->CLM1 + ADC0->CLM2 + ADC0->CLM3 + ADC0->CLM4 +
	ADC0->CLMS;
	cal_v = cal_v >> 1 | 0x8000;
	ADC0->MG = cal_v;
	ADC0->SC3 = 0; // Turn off Hardware Averaging
}


void sense()
{
	unsigned int light_val_L = 0;
	unsigned int light_val_R = 0;
	coverL=0;
	while(coverL<260){
		sonar();
		controlMotor(1,1,4500);
		controlMotor(2,1,4600);

		ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		light_val_L = ADC0->R[0]; // Resets COCO

		ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		light_val_R = ADC0->R[0]; // Resets COCO

		if(light_val_R > 240)
		{
			turnRightAmount(10);
		}
		else if(light_val_L > 240)
		{
			turnLeftAmount(10);
		}
	}
	turnLeft();
	delay_ms(20);
	coverL=0;
	while(coverL<250)
	{
		sonar();
		controlMotor(1,1,4400);
		controlMotor(2,1,4500);

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
				turnLeftAmount(130);
				break;
			}
			else if(light_val_L > 240)
			{
				turnLeftAmount(130);
				break;
			}
		}

	}
	coverL=0;
	while(coverL<300){
		sonar();
		controlMotor(1,1,4500);
		controlMotor(2,1,4600);

		ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		light_val_L = ADC0->R[0]; // Resets COCO

		ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		light_val_R = ADC0->R[0]; // Resets COCO

		if(light_val_R > 240)
		{
			turnLeftAmount(15);
		}
		else if(light_val_L > 240)
		{
			turnLeftAmount(15);
		}
	}


}




void checkReflect()
{
	initADC();
	unsigned int light_val_L = 0;
	unsigned int light_val_R = 0;
	while(1)
	{
		delay_s(2);
		ADC0->SC1[0] = 0x01; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		light_val_L = ADC0->R[0]; // Resets COCO
		printf("Light_val_L = %d\n",light_val_L);

		ADC0->SC1[0] = 0x05; // Set Channel, starts conversion.
		while(!(ADC0->SC1[0] & 0x80)){ }
		light_val_R = ADC0->R[0]; // Resets COCO
		printf("Light_val_R = %d\n",light_val_R);
	}
}

void autoDrive()
{
	//sonar();
	sense();
	//controlMotor(1,1,5500);
	//controlMotor(2,1,5600);
}

int main(void) {

  	/* Init board hardware. */

	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    PRINTF("Board Initialized\n");
    setupSW1();
    PRINTF("Press SW1 to begin.\n");

    while(!SW1Pressed()){};
    delay_s(2);

    initMotorControl();
    printf("Motor Control Initialized\n");
    initSpeedControl();
    printf("Speed Control Initialized\n");
    initObstacle();
    printf("Obstacle Avoidance Initialized\n");
    delay_ms(500);
    //turnRight();
    //sonar();

    initADC();
    //checkReflect();
    //turnLeftAmount(50);
    //turnRightAmount(50);
    autoDrive();

    return 0 ;
}
