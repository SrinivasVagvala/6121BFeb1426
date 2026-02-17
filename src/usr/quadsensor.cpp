#include "main.h"
#include "quadsensor.hpp"


RobotPos rohan;


// Update these to match your actual ports
pros::Distance frontD(FRONTDISTANCE);
pros::Distance backD(BACKDISTANCE);
pros::Distance leftD(LEFTDISTANCE);
pros::Distance rightD(RIGHTDISTANCE);

// add sensor variables here
float sensorNoise  = 0.5;   
float processNoise = 0.1;   
float fieldSize    = 140.5; 

// set robot starting position to (0,0,0) so the coordinate is relative to the 
RobotPos::RobotPos() {
    X[0] = 0; X[1] = 0; X[2] = 0; 
    confX = 1.0; confY = 1.0;
}

void RobotPos::updateAxis(float& currentPos, float& currentConf, float measurement) {
    // as the bot moves the confidence decrease so we have noise increase
    currentConf += processNoise;

    // calculate how much to trust the sensor (0.0 to 1.0)
    float K = currentConf / (currentConf + sensorNoise);

    // correct our position estimate towards the measurement based on confidence
    currentPos = currentPos + K * (measurement - currentPos);

    // if we see a wall increase confidence
    currentConf = (1.0 - K) * currentConf;
}

void RobotPos::update(float f_raw, float b_raw, float l_raw, float r_raw) {
    float f = f_raw * mmToIn;
    float b = b_raw * mmToIn;
    float l = l_raw * mmToIn;
    float r = r_raw * mmToIn;

    // y-Axis (forward/back)
    if (f_raw > 20 && f_raw < 2000) updateAxis(X[1], confY, fieldSize - f);
    else if (b_raw > 20 && b_raw < 2000) updateAxis(X[1], confY, b);

    // x-Axis (left/right)
    if (l_raw > 20 && l_raw < 2000) updateAxis(X[0], confX, l);
    else if (r_raw > 20 && r_raw < 2000) updateAxis(X[0], confX, fieldSize - r);
}

void posTask(void* parameter) {
    while (true) {
        rohan.update(frontD.get(), backD.get(), leftD.get(), rightD.get());

        chassis.odom_xyt_set((double)rohan.X[0], (double)rohan.X[1], (double)rohan.X[2]);

        if (!pros::competition::is_autonomous()) {
            posOpControl();
        }
         
        pros::delay(10); 
    }
}



void posOpControl() {
    pros::lcd::print(0, "X: %.2f in", rohan.X[0]);
    pros::lcd::print(1, "Y: %.2f in", rohan.X[1]);
}



void distanceCorrection(int targetD, int distFromWall, int speed, SensorSide side) {

    chassis.pid_drive_set(targetD, speed, true);
    chassis.pid_wait();

    if (side == FRONT) {
        float dtm = frontD.get() - distFromWall;

        chassis.pid_drive_set(dtm, speed, true);
        chassis.pid_wait();
    }
    else if (side == BACK) {
        float dtm = backD.get() - distFromWall;

        chassis.pid_drive_set(dtm, speed, true);
        chassis.pid_wait();
    } 

}

