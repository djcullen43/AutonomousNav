/*
 * SpeedControl.h
 *
 *  Created on: Oct 24, 2022
 *      Author: djcullen
 */
extern volatile int distL;
extern volatile int distR;
extern volatile int distM;
extern volatile int lastR;
extern volatile int lastL;
extern volatile int coverL;
extern volatile int coverR;
extern volatile float timeElapsed;
void initSpeedControl(void);
void initPIT(void);
void PIT_IRQHandler(void);
void PORTA_IRQHandler(void);


