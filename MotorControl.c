/*
 * MotorControl.c
 *
 *  Created on: Oct 5, 2022
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

//Initilizes all port required for motor control and enables PWM
//PTB0, PTB1, PTB2, PTB3, PTC1 and PTC2 signals. Note that PTB2 and PTB3 will be driven by TPM2 in PWM mode, so their muxes should be set accordingly.  The others should be set up as GPIO outputs.
void initMotorControl()
{
	//Enable clock gate for port B
	SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK;
	//Set ports for GPIO PTB0
	PORTB->PCR[0] &= ~PORT_PCR_MUX_MASK ;
	PORTB->PCR[0] = PORT_PCR_MUX(1u);
	GPIOB->PDDR |= (1 << 0);
	//Set ports for GPIO PTB1
	PORTB->PCR[1] &= ~PORT_PCR_MUX_MASK ;
	PORTB->PCR[1] = PORT_PCR_MUX(1u);
	GPIOB->PDDR |= (1 << 1);

	//Set ptb1 and ptb2 to be driven by tpm0
	PORTB->PCR[2] &= ~PORT_PCR_MUX_MASK ; // Clear MUX
	PORTB->PCR[2] |= 0x300; // Drive Pin with TPM2
	PORTB->PCR[3] &= ~PORT_PCR_MUX_MASK ; // Clear MUX
	PORTB->PCR[3] |= 0x300; // Drive Pin with TPM2

	//Setup the pwm prescaler 1 at 1KHz
	SIM->SCGC6 |= (1<<26); //Clk enable tpm2

	//Enable clock gate for port C
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	SIM->SOPT2 |= (0x2<<24); //Use OSCERCLK]
	TPM2->SC = (0x1<<5) | (0x1<<7)| (1<<0);//set CPWMS to 0, reset timer overflow, set prescaler to 1
	TPM2->MOD =7999; //set mod to 7999 as 8MHz/8 = 1KHz so 125Khz/125 = Khz;
	TPM2->CONTROLS[0].CnSC |= (0x2 << 2) | (0x2 << 4);  // Edge-aligned high-true PWM Channel 0
	TPM2->CONTROLS[1].CnSC |= (0x2 << 2) | (0x2 << 4);  // Edge-aligned high-true PWM Channel 1
	TPM2->CONF |= (0x3<<6); //Enable debug mode for tpm2
	TPM2->CONTROLS[0].CnV = 7999;//ptb2
	TPM2->CONTROLS[1].CnV = 7999;//ptb3
	TPM2->SC |= (1<<7);

	TPM2->SC |= 0x01<<3;

	//Set ports for GPIO PTC1
	PORTC->PCR[1] &= ~PORT_PCR_MUX_MASK ;
	PORTC->PCR[1] = PORT_PCR_MUX(1u);
	GPIOC->PDDR |= (1 << 1);
	//Set ports for GPIO PTC2
	PORTC->PCR[2] &= ~PORT_PCR_MUX_MASK ;
	PORTC->PCR[2] = PORT_PCR_MUX(1u);
	GPIOC->PDDR |= (1 << 2);
}


//Left motor forward
void FML(int speed)
{
	GPIOB->PDOR &= ~(1<<1);//in1
	GPIOB->PDOR |= (1<<0);//in2

	TPM2->CONTROLS[0].CnV = speed;//ptb2
}

//left motor reverse
void RML(int speed)
{
	GPIOB->PDOR |= (1<<1);//in1
	GPIOB->PDOR &= ~(1<<0);//in2

	TPM2->CONTROLS[0].CnV = speed;//ptb2
}

//right motor forward
void FMR(int speed)
{
	GPIOC->PDOR |= (1<<1); //in1
	GPIOC->PDOR &= ~(1<<2); //in2

	TPM2->CONTROLS[1].CnV = speed;//ptb3
}

//right motor reverse
void RMR(int speed)
{
	GPIOC->PDOR &= ~(1<<1);//in1
	GPIOC->PDOR |= (1<<2);//in2

	TPM2->CONTROLS[1].CnV = speed;//ptb3
}


//stop right motor
void STR()
{
	GPIOC->PDOR &= ~(1<<1);//in1
	GPIOC->PDOR &= ~(1<<2);//in2

	TPM2->CONTROLS[1].CnV = 7999;//ptb2
}

//stop left motor
void STL()
{
	GPIOB->PDOR &= ~(1<<1);//in1
	GPIOB->PDOR &= ~(1<<0);//in2

	TPM2->CONTROLS[0].CnV = 7999;//ptb2
}

//stop all motors
void AllStop()
{
	STL();
	STR();
}


//Motor controller function which drives the motors with specified inputs.
// Motor 1=left 2=right:::Direction 1=forward 2=backward :::speed = 0-100
void controlMotor(int motor, int direction, int speed)
{
	if(motor==1 && direction==1) //Forward left
	{
		FML(speed);
	}
	else if(motor==1 && direction==2)//reverse left
	{
		RML(speed);
	}
	else if(motor==2 && direction==1)//forward right
	{
		FMR(speed);
	}
	else if(motor==2 && direction==2)//reverse right
	{
		RMR(speed);
	}
	return;
}


//turn left 90 degrees
void turnL()
{
	controlMotor(2,1,80);//FR
	delay_s(1);
	delay_ms(450);
	AllStop();
}

//turn right 90 degrees
void turnR()
{
	controlMotor(1,1,80);//FR
	delay_s(1);
	delay_ms(450);
	AllStop();
}
