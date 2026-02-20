#ifndef _QUADSENSOR_HPP_
#define _QUADSENSOR_HPP_

#include "main.h"

class RobotPos {
private:
    const float mmToIn = 1.0f / 25.4f;

    // confidence in our x and y values
    float confX;
    float confY;

    // blends guess with sensor data using confidence and noise values
    void updateAxis(float& currentPos, float& currentConf, float measurement);

public:
    // field position in inches (x, y, heading) = (0,1,2)
    float X[3];

    RobotPos();

    // use sensor data to update our position estimate
    void update(float f_raw, float b_raw, float l_raw, float r_raw);
};

void posOpControl();
void posTask(void* parameter);

void quadSensorTest();

enum SensorSide { FRONT, BACK };

void distanceCorrection(int targetD, int distFromWall, int speed, SensorSide side);

extern RobotPos rohan;

#endif 