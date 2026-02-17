#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <sys/_intsup.h>
#include "middlegoal.hpp"

pros::Motor lowerintake(LOWER_INTAKE);
pros::Motor scoring(TOP_INTAKE);

pros::adi::Pneumatics scorePiston(SCORING, true, true);
pros::adi::Pneumatics ballLock(BALLLOCK, true, true);
pros::adi::Pneumatics scoringState(SCORESTATE, true, true);


extern pros::adi::Pneumatics middleGoal;


pros::adi::Pneumatics midDescore(MIDDESCORE, true, true);

extern pros::Distance frontD;


bool redTeam = true; // true signifies red team, so color sorting blue, and vice versa if false
bool match = false;

bool antiJamOn = false;


int intakeStuckTime = 0;


int lowerVelocity = 100;
int scoringVelocity = 100;


int overHeatTemp = 55; // temperature at which the motor is considered overheated

int autonLowerVelocity = 100;
int autonScoringVelocity = 100;

void reverseScoring(bool state){
    if (state){
        scoring.move(autonScoringVelocity*1.27*0.3);
    }
    else{
        scoring.move(0);
    }
}


void intake(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*1.2);
    }
    else{
        lowerintake.move(0);
    }
}

void extake(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*-1);
        scoring.move(autonScoringVelocity);
    }
}

void scoreHigh(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*1.27);
        scoring.move(autonScoringVelocity*-1.27);
    }
}

void descoreMidToggle(){
    midDescore.toggle();
}

void scoreMid(bool state, bool useExtake){ // sometimes need extake
    if (state){

        if (useExtake){
            reverseScoring(true);
            extake(true);
            
            pros::delay(100);

            lowerintake.move(autonLowerVelocity*0.8255);
            scoring.move(scoringVelocity*-1.016);
        }
        else{
            lowerintake.move(autonLowerVelocity*0.90);
            scoring.move(scoringVelocity*-0.7);
        }
    }
    else{
        lowerintake.move(0);
        scoring.move(0);

    }
}

void scoreLow(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*-0.7);
        scoring.move(autonScoringVelocity*1.27);
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }
}

void MiddleAlignerActive(bool state){
    if(state){
        middleGoalPiston(true);
    }

}

void MiddleAlignerInactive(bool state){
    if(state){
        middleGoalPiston(false);
    }

}

void MiddleGoalScoreSkills(bool state){
    if(state){
        lowerintake.move(autonLowerVelocity*0.65);
        scoring.move(autonScoringVelocity*-0.85);
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }

}

void intakeOpControl(){  // the intake velocity switches based on which button is being pressed
    if(lowerintake.get_actual_velocity() > 0 && matchloadExtended()==false && scoring.get_actual_velocity()==0 && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
        
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {  // intake

        lowerintake.move(lowerVelocity*1.10);
        scoring.move(scoringVelocity*-1.10);
        ballLock.extend();
        scoringState.extend();
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) { 
        // used in descore.cpp for wings
    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) { // low goal scoring / extake
        
        scorePiston.extend();  
        lowerintake.move(lowerVelocity*-1.27);
        scoring.move(scoringVelocity*1.27);
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) { // scoring
        if(scoringState.is_extended()){ // long goal scoring
            lowerintake.move(lowerVelocity*1.27);
            scoring.move(scoringVelocity*-1.27);

        }
        else if(scoringState.is_extended() == false){ // middle goal scoring
            lowerintake.move(lowerVelocity*1.17);
            scoring.move(scoringVelocity*-1.17);
        }
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) { // middle goal descore
        midDescore.toggle();
    }
    else { // if no buttons are being pressed, the motors will stop
        lowerintake.move(0);
        scoring.move(0);
    }
}

void heatSensingControl() {
    // intake heat sensing code
    int lowerHeat = lowerintake.get_temperature();
    int scoringHeat = scoring.get_temperature();

    if (lowerHeat >= overHeatTemp) {
        master.print(1,0,"Lower Intake Overheated");
    }
    if (scoringHeat >= overHeatTemp) {
        master.print(1,0,"Scoring Intake Overheated");
    }

    // chassis temperature code

    // left sides
    int LF = chassis.left_motors[-1].get_temperature();
    int LM = chassis.left_motors[-2].get_temperature();
    int LB = chassis.left_motors[-3].get_temperature();
    // right sides
    int RF = chassis.right_motors[11].get_temperature();
    int RM = chassis.right_motors[12].get_temperature();
    int RB = chassis.right_motors[13].get_temperature();

    if (LF >= overHeatTemp) {
        master.print(2,0,"Left Front Overheated");
    }
    if (LM >= overHeatTemp) {
        master.print(2,0,"Left Middle Overheated");
    }
    if (LB >= overHeatTemp) {
        master.print(2,0,"Left Back Overheated");
    }
    if (RF >= overHeatTemp) {
        master.print(2,0,"Right Front Overheated");
    }
    if (RM >= overHeatTemp) {
        master.print(2,0,"Right Middle Overheated");
    }
    if (RB >= overHeatTemp) {
        master.print(2,0,"Right Back Overheated");
    }
}

void setMatch(bool state){
    match = state;
}

void setTeam(bool state){
    redTeam = state;
}

void turnOffLower(bool state){
    if (state){
        lowerintake.move(0);
    }
}

void turnOffScoring(bool state){
    if (state){
        scoring.move(0);
    }
}

void turnLoweratXSpeed(bool state,int speed){
    if (state){
        lowerintake.move(speed);
    }
}

void turnScoringatXSpeed(bool state, int speed){
    if (state){
        scoring.move(speed);
    }
}

void matchLoadSorting() {

    // int lowerOptical_bright = 75;

    // lowerOptical.set_led_pwm(lowerOptical_bright);

    // double hue2 = lowerOptical.get_hue();
    
    // master.print(0,0,"Hue   %f",hue2);

    // // benchmark values for each ring, for each parameter
    // double bHue = 46;

    // double rHue = 40;


    
    // if ((hue2 > bHue) && redTeam) { 
    // master.print(0,0,"BLU"); 
    // lowerintake.move(autonLowerVelocity*-1);
    // pros::delay(300);
    // lowerintake.move(autonLowerVelocity*1);    
    // }

    // else if((hue2 < rHue) && !redTeam){
    // master.print(0,0,"RED");
    // lowerintake.move(autonLowerVelocity*-1);
    // pros::delay(340);
    // lowerintake.move(autonLowerVelocity*1);
    // }
    

     
}

void antiJam(){
    if (fabs(lowerintake.get_actual_velocity()) < 0.5 && fabs(lowerintake.get_voltage()) > 2000) {
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 20) {
            master.print(0,0, "This happens");
            master.rumble("-");
            lowerintake.move(autonLowerVelocity*-1);
            pros::delay(50);
            lowerintake.move(autonLowerVelocity);
            intakeStuckTime = 0;
        }
    }

    if (fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000) { // checks if scoring rollers are jammed
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "This happens");
            master.rumble("-");
            scoring.move(autonScoringVelocity);
            pros::delay(100);
            scoring.move(-1 * autonScoringVelocity);
            intakeStuckTime = 0;
        }
    }


}

void setAntiJam(bool state){
    antiJamOn = state;
}

void intakeTask(void* parameter) {
    lowerintake.tare_position();
    scoring.tare_position();
    
    while (true) {
        if (!pros::competition::is_autonomous()) { // if the intake is not in the autonomous state ex: op control
            lowerintake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            scoring.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            intakeOpControl();
            //heatSensingControl();

            // master.clear();

            // pros::delay(50);

            // master.print(0,0,"Front: %i",frontD.get());

            // pros::delay(1000);
            
        }
        else if (pros::competition::is_autonomous()) {
            //heatSensingControl();
            if(lowerintake.get_actual_velocity() > 0 and match){
                matchLoadSorting();
            }
            if(antiJamOn){
                antiJam();
            }

            

            // master.clear();

            // pros::delay(50);

            // master.print(0,0,"Front: %i",frontD.get() - 287);

            // pros::delay(50);

            // master.print(1,0,"Back: %i",backD.get() - 104);

            // pros::delay(1000);

        }

        pros::delay(20);
    }
}








