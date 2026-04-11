#include "EZ-Template/util.hpp"
#include "main.h"
#include "quadsensor.hpp"





pros::Distance frontD(FRONTDISTANCE);
pros::Distance backD(BACKDISTANCE);
pros::Distance leftD(LEFTDISTANCE);
pros::Distance rightD(RIGHTDISTANCE);


bool useF = true, useR = true; // decides if using front or back distance sensor, and right or left distance sensor

float x = 0, y = 0; // x and y for putting into odom.xyt.reset()

float dFV = 0, dFR = 0; // dfv is distance from vertical wall, we are assuming vertical wall is front so we will calculate it if its back, same thing with dfR but for right wall

void posTask(void* parameter) {



    while (true) {

        if (!pros::competition::is_autonomous()) {

            master.print(0,0,"Front: %i",frontD.get());

            posOpControl();

        }
         
        pros::delay(10); 
    }
}



void posOpControl() {
    getLocation();
}

void getLocation() {
    int frontDistance = frontD.get();
    int backDistance = backD.get();
    int leftDistance = leftD.get();
    int rightDistance = rightD.get();

    if (frontDistance > 3000) {
        useF = false;
        master.print(0,0, "D: %i", backDistance);
    }
    else {
        useF = true;
        master.print(0,0, "D: %i", frontDistance);
    }

    if (rightDistance > 3000) {
        useR = false;
        master.print(0,0, "D: %i", leftDistance);
    }
    else {
        useR = true;
        master.print(0,0, "D: %i", rightDistance);
    }

    
    



    
}

