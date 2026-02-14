#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/misc.h"
#include <sys/_intsup.h>
#include "middlegoal.hpp"
#include "pros/rtos.hpp"

pros::adi::Pneumatics middleGoal(MIDDLEGOAL, true, true);

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
        //middle goal control code here
    if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
        middleGoal.retract();
    }
    else{
        middleGoal.extend();
    }
}
void middleGoalPiston(bool state){
    if(state){
        middleGoal.retract();
    }
    else{
        middleGoal.extend();
    }
}