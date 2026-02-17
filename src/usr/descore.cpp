#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/misc.h"
#include <sys/_intsup.h>


pros::adi::Pneumatics descore(WING, false, true);



void descoreOpControl(){  // the intake velocity switches based on which button is being pressed
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) { // if l2 is pressed the wings are activated
        descore.toggle();

    }
} 


void wingToggle(){
    descore.toggle();
}

void descoreTask(void* parameter) {
    
    while (true) {
        if (!pros::competition::is_autonomous()) { // if the intake is not in the autonomous state ex: op control
            descoreOpControl();
        }
        else if (pros::competition::is_autonomous()) {
        }

        pros::delay(20);
    }
}








