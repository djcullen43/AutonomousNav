/*
 * MotorControl.h
 *
 *  Created on: Oct 5, 2022
 *      Author: djcullen
 */

void initMotorControl();
void controlMotor(int motor, int direction, int speed);
void FML(int speed);
void FMR(int speed);
void RML(int speed);
void RMR(int speed);
void turnL();
void turnR();
void STL();
void STR();
void AllStop();



