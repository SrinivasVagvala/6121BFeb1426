#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/misc.h"
#include <sys/_intsup.h>
#include "middlegoal.hpp"
#include "pros/rtos.hpp"

extern pros::adi::Pneumatics scoringState;
extern pros::adi::Pneumatics ballLock;
extern pros::adi::Pneumatics scorePiston;
extern pros::adi::Pneumatics midDescore;

extern bool ball_lock;   

void middleGoalTask(void *parameter){
    
    while (true){
        if (!pros::competition::is_autonomous()){
            middleGoalOpControl();
        }
        else{
            pros::delay(0);
        }
    }
}

void middleGoalOpControl(){
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){

        ball_lock = true;

        ballLock.extend();
        scoringState.extend();
        scorePiston.extend();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){ // middle goal descore
        midDescore.toggle();
    }

    if (not master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && not master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && not master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && not master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){

        ball_lock = false;
        
        
        scoringState.retract();
        scorePiston.retract();
    }
    
}
void middleGoalPiston(bool state){
    if(state){
        scoringState.extend();
        ballLock.retract();
        scorePiston.retract();
    }
    else{
        scoringState.retract();
        ballLock.extend();
        scorePiston.extend();
    }
}