/*
 * Utilities.c
 *
 *  Created on: Oct 5, 2022
 *      Author: djcullen
 */
#include <stdbool.h>
#include<stdio.h>
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKL46Z4.h"
#include "fsl_debug_console.h"
#include "MotorControl.h"
#include "Utilities.h"

//initializes sw1
void setupSW1()
{
	SIM->SCGC5 |= SIM_SCGC5_PORTC_MASK;
	PORTC->PCR[3] &= ~(0x703); // Clear mux and PE/PS bits
	PORTC->PCR[3] |= 0x703 & ((1 << 8) | 0x03); // Set MUX bits to GPIO, Set pullup and pull enable.
	GPIOC->PDDR &= ~(1 << 3); // Clear Data direction (input)
}
//checks if sw1 is pressed
bool SW1Pressed()
{
	if(!(GPIOC->PDIR & 0x8))
	{
		return true;
	}
	return false;;
}

//delay function up to 1000ms
void delay_ms(unsigned short delay_t) {
    SIM->SCGC6 |= (1 << 24); // Clock Enable TPM0
    SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
    TPM0->CONF |= (0x1 << 17); // Stop on Overflow
    TPM0->SC = (0x1 << 7) | (0x07); // Reset Timer Overflow Flag, Set Prescaler 128
    TPM0->MOD = delay_t * 61 + delay_t/2; //

    TPM0->SC |= 0x01 << 3; // Start the clock!

    while(!(TPM0->SC & 0x80)){} // Wait until Overflow Flag
    return;
}
//delay function up to 1000ms
void delay_us(unsigned short delay_t) {
    SIM->SCGC6 |= (1 << 24); // Clock Enable TPM0
    SIM->SOPT2 |= (0x2 << 24); // Set TPMSRC to OSCERCLK
    TPM0->CONF |= (0x1 << 17); // Stop on Overflow
    TPM0->SC = (0x1 << 7) | (0x03); // Reset Timer Overflow Flag, Set Prescaler 8
    TPM0->MOD = delay_t-1; //set the MOD value to

    TPM0->SC |= 0x01 << 3; // Start the clock!

    while(!(TPM0->SC & 0x80)){} // Wait until Overflow Flag
    return;
}
//delay function for specified # of seconds
void delay_s(unsigned short seconds) {
	unsigned short counter = seconds;
	while (counter!=0)
	{
		delay_ms(1000);
		counter--;
	}
    return;
}
