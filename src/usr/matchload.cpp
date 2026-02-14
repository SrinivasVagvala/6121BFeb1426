#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.h"
#include <sys/_intsup.h>

pros::adi::Pneumatics MATCHLOADER (MATCHLOAD, true, true);



void matchloaderOpControl(){
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
        MATCHLOADER.toggle();
    }

}

void matchloaderInactive(bool state){
    if(state){
        MATCHLOADER.extend();
    }

}

bool matchloadExtended(){
    return MATCHLOADER.is_extended();
}

void matchloaderActive(bool state){
    if (state){
        MATCHLOADER.retract();
    }

}


void matchloaderTask(void* parameter) {
    
    while (true) {
        if (!pros::competition::is_autonomous()) { // if the intake is not in the autonomous state ex: op control
            matchloaderOpControl();
        }
        else if (pros::competition::is_autonomous()) {
        }

        pros::delay(20);
    }
}