#ifndef _QUADSENSOR_HPP_
#define _QUADSENSOR_HPP_

#include "main.h"



void posOpControl();
void posTask(void* parameter);


enum SensorSide { FRONT, BACK };

void distanceCorrection(int targetD, int distFromWall, int speed, SensorSide side);


#endif 