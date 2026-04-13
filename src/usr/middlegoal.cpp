#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/misc.h"
#include <sys/_intsup.h>
#include "middlegoal.hpp"
#include "pros/rtos.hpp"

extern pros::adi::Pneumatics midDescore;

extern bool ball_lock;  
extern bool tester; 

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
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ // middle goal piston movements

        ball_lock = true;
        //midDescore.retract();

    }

    if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ // middle goal piston movements

        ball_lock = false;
        //midDescore.extend();

    }


    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)){ // middle goal descore
        midDescore.retract();
    }
    
    if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_Y) && tester == false){
        midDescore.extend();
    }

    if(tester){
        midDescore.retract();
    }



    if (not master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && not master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && not master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
        ball_lock = false;
         
    }



    
}
void middleGoalPiston(bool state){ // done
    if(state){
    }
    else{
    }
}

void midDescoreToggle(){ // done
    midDescore.toggle();
}