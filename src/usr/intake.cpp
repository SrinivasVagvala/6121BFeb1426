#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <sys/_intsup.h>
#include "middlegoal.hpp"

pros::Motor lowerintake(LOWER_INTAKE);
pros::Motor scoring(TOP_INTAKE);

pros::adi::Pneumatics scorePiston(SCORING, true, true); // 

extern pros::adi::Pneumatics middleGoal;
extern pros::adi::Pneumatics descore;

pros::adi::Pneumatics midDescore(MIDDESCORE, true, true);

extern pros::Distance frontD;


bool redTeam = true; // true signifies red team, so color sorting blue, and vice versa if false
bool match = false;

bool antiJamOn = false;
bool antiState = false;

bool stopScoring = false;
bool ball_lock = false;

bool buttonDone = false;


int intakeStuckTime = 0;


int lowerVelocity = 100;
int scoringVelocity = 100;


int overHeatTemp = 55; // temperature at which the motor is considered overheated


int autonLowerVelocity = 100;
int autonScoringVelocity = 100;

void reverseScoring(bool state){
    if (state){
        scoring.move(autonScoringVelocity*1.27);
    }
    else{
        scoring.move(0);
    }
}


void intake(bool state){
    if (state){
        scoring.move(autonScoringVelocity*-1.27);
        lowerintake.move(autonLowerVelocity*1.27);
        
        descore.extend();
        scorePiston.extend();
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }
}

void extake(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*-1.27);
        scoring.move(autonScoringVelocity*1.27);
    }
}

void scoreHigh(bool state){
    if (state){
        scoring.move(autonScoringVelocity*-1.27);
        lowerintake.move(autonLowerVelocity*1.27);

        descore.retract();

        scorePiston.extend();
    }
}



void scoreMid(bool state, bool useExtake){ // sometimes need extake
    if (state){

        if (useExtake){
            reverseScoring(true);
            extake(true);
            
            pros::delay(100);

            scoring.move(autonScoringVelocity*-1.27);
            lowerintake.move(autonLowerVelocity*1.27);
        }
        else{
            scoring.move(autonScoringVelocity*-1.27*0.85);
            lowerintake.move(autonLowerVelocity*1.27);

        }
    }
    else{
        lowerintake.move(0);
        scoring.move(0);

    }
}

void scoreLow(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*-1.27);
        scoring.move(autonScoringVelocity*1.27);
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }
}



void MiddleGoalScoreSkills(bool state){
    if(state){
        scoring.move(autonScoringVelocity*-1.27*0.65);
        lowerintake.move(autonLowerVelocity*1.27*0.65);
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }

}

void intakeOpControl(){  // the intake velocity switches based on which button is being pressed
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ){ // low goal / extake
        lowerintake.move(lowerVelocity*-0.70);//120 for matches 70 for skills
        scoring.move(scoringVelocity*1.27);

    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2)){
        stopScoring = false;   
    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && buttonDone == false) { // intake
        buttonDone = true;

        
        lowerintake.move(lowerVelocity*1.27);
        

        if (stopScoring == false){
            scoring.move(scoringVelocity*-1.27);
        }
        else {
            scoring.move(0);
        }

        
        descore.extend();
        scorePiston.extend();

    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && buttonDone ){

        lowerintake.move(lowerVelocity*1.27);
        

        if (stopScoring == false){
            scoring.move(scoringVelocity*-1.27);
        }
        else {
            scoring.move(0);
        }

    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ){ // scoring
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ //midgoal
            scoring.move(scoringVelocity*-1.27*0.68);// 68 for skills 90 for regular
            lowerintake.move(lowerVelocity*1.27);

            descore.retract();

        }
        else { //long goal
            scoring.move(scoringVelocity*-1.27);
            lowerintake.move(lowerVelocity*1.27);

            descore.retract();

            scorePiston.extend();
        }

    }
    else {
        lowerintake.move(0);
        scoring.move(0);
        buttonDone = false;
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
        scoring.move(speed*-1);
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

void antiJam(bool direction){// intake is true
    if (fabs(lowerintake.get_actual_velocity()) < 0.5 && fabs(lowerintake.get_voltage()) > 2000) {
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 20 && direction) {
            master.print(0,0, "This happens");
            master.rumble("-");
            lowerintake.move(autonLowerVelocity*-1);
            pros::delay(50);
            lowerintake.move(autonLowerVelocity);
            intakeStuckTime = 0;
        }
        else if (pros::millis() - intakeStuckTime > 20 && direction==false) {
            master.print(0,0, "This happens");
            master.rumble("-");
            lowerintake.move(autonLowerVelocity);
            pros::delay(50);
            lowerintake.move(autonLowerVelocity*-1);
            intakeStuckTime = 0;
        }
    }

    if (fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000) { // checks if scoring rollers are jammed
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200 && direction) {
            master.print(0,0, "This happens");
            master.rumble("-");
            scoring.move(autonScoringVelocity);
            pros::delay(100);
            scoring.move(-1 * autonScoringVelocity);
            intakeStuckTime = 0;
        }

        else if (pros::millis() - intakeStuckTime > 200 && direction == false) {
            master.print(0,0, "This happens");
            master.rumble("-");
            scoring.move(autonScoringVelocity*-1);
            pros::delay(100);
            scoring.move(autonScoringVelocity);
            intakeStuckTime = 0;
        }
        
    }

}

void scoringJam(){
    if (fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000) { // checks if scoring rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "stop scoring");
            master.rumble("-");
            scoring.move(0);

            stopScoring = true;
            intakeStuckTime = 0;
        }
    }
}

void setAntiJam(bool state, bool antiS){
    antiJamOn = state;
    antiState = antiS;
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

            scoringJam();

            // master.print(0,0,"Front: %i",frontD.get());

            // pros::delay(1000);

            
        
            
        }
        else if (pros::competition::is_autonomous()) {
            //heatSensingControl();
            if(lowerintake.get_actual_velocity() > 0 and match){
                matchLoadSorting();
            }
            if(antiJamOn){
                if(antiState){
                    antiJam(true);
                }
                else{
                    antiJam(false);
                }
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








