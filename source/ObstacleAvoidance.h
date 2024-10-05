/*
 * ObstacleAvoidance.h
 *
 *  Created on: Nov 14, 2022
 *      Author: 43lio
 */
extern volatile int endCnt;
extern volatile int startCnt;

extern bool startFlag;
void initSensor();
void initServo();
void initObstacle();
void moveServo(float degree);
void turnLeftAmount(int amount);
void turnRightAmount(int amount);

int pulse();
void sonar();
void turnLeft();
void turnRight();
void uturn();
