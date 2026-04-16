#include "EZ-Template/util.hpp"
#include "globals.hpp"
#include "main.h"
#include "pros/adi.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include <sys/_intsup.h>
#include "middlegoal.hpp"
#include "pros/rtos.hpp"

pros::Motor lowerintake(LOWER_INTAKE); // positive is extake, negative is intake
pros::Motor scoring(TOP_INTAKE); // positive is scoring, negative is extaking and middle goal
pros::Motor middle(MIDDLE_INTAKE); // positive is extake, negative is intake 

    
extern pros::adi::Pneumatics descore;

pros::adi::Pneumatics midDescore(MIDDESCORE, true, true);
pros::adi::Pneumatics hood(HOOD, false, false);

// pros::Distance frontD(FRONTDISTANCE);
// pros::Distance backD(BACKDISTANCE);
// pros::Distance leftD(LEFTDISTANCE);
// pros::Distance rightD(RIGHTDISTANCE);

bool redTeam = true; // true signifies red team, so color sorting blue, and vice versa if false
bool match = false;

bool antiJamOn = false;
bool antiState = false;

bool stopScoring = false;
bool stopMiddle = false;
bool stopMiddleLow = false;
bool ball_lock = false;

bool buttonDone = false;


int intakeStuckTime = 0;


int lowerVelocity = 100;
int scoringVelocity = 100;
int midVelocity = 100;


int overHeatTemp = 55; // temperature at which the motor is considered overheated


int autonLowerVelocity = 100;
int autonScoringVelocity = 100;

bool tester = true; // call Srinivas if you want to ask abt this variable

bool AntiJamActiveWorlds = false; 

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
        
        hood.retract();
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


        hood.extend();
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

void fastSkillsMidScore(bool state){ // sometimes need extake
    if (state){
        scoring.move(autonScoringVelocity*-1.27*0.75);
        lowerintake.move(autonLowerVelocity*1.27);

    }
    else{
        lowerintake.move(0);
        scoring.move(0);

    }
}

void scoreLow(bool state){
    if (state){
        lowerintake.move(autonLowerVelocity*-1.27*0.75);
        scoring.move(autonScoringVelocity*1.27*0.75);
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }
}

void MiddleGoalScoreSkills(bool state){
    if(state){
        scoring.move(autonScoringVelocity*-1.27*0.55);
        lowerintake.move(autonLowerVelocity*1.27);
    }
    else{
        lowerintake.move(0);
        scoring.move(0);
    }

}

void intakeOpControl(){  // the intake velocity switches based on which button is being pressed
    

    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ){ // low goal / extake for skills
        
        lowerintake.move(lowerVelocity*1.27*0.8);
            
        middle.move(midVelocity*1.27*0.8);
            
        scoring.move(scoringVelocity*1.27*0.8);


    }
    // if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B) ){ // low goal / extake
        
    //     lowerintake.move(lowerVelocity*1.27);
            
    //     middle.move(midVelocity*1.27);
            
    //     scoring.move(scoringVelocity*1.27);


    // }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && buttonDone == false) { // intake

            lowerintake.move(lowerVelocity*-1.27);
            
            if (stopMiddle == false){
                middle.move(midVelocity*-1.27);
            }
            else{
                middle.move(0);
            }

            if (stopScoring == false){
                scoring.move(scoringVelocity*-1.27);
            }
            else {
                //scoring.move(0);
            }

            
            hood.retract();
       ///}
    }
    else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && buttonDone ){

        lowerintake.move(lowerVelocity*-1.27);
        
        if (stopMiddle == false){
            middle.move(midVelocity*-1.27);
        }
        else{
            middle.move(0);
        }

        if (stopScoring == false){
            scoring.move(scoringVelocity*-1.27);
        }
        else {
            //scoring.move(0);
        }
    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)){ // low goal extake for 50 mils before long scoring

        lowerintake.move(lowerVelocity*1.27);

        pros::delay(50);

    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) ){ //long goal

        scoring.move(scoringVelocity*-1.27);
        lowerintake.move(lowerVelocity*-1.27);
        middle.move(midVelocity*-1.27);
        hood.extend();

    }
    else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)){ // needs to extake balls a bit for the middle goal scoring

        // midDescore.extend();

        middle.move(midVelocity*1.27);
            
        lowerintake.move(scoringVelocity*1.27);

        scoring.move(scoringVelocity*1.27);

        pros::delay(300);

        tester = true;

    }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ // middle goal scoring 
            scoring.move(scoringVelocity*1.27);

            if (stopMiddleLow == false){
                lowerintake.move(lowerVelocity*-1.17);
                middle.move(midVelocity*-0.75);
            }

    }
    // else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){ // scores 7 balls for skills 
    //         scoring.move(scoringVelocity*0.87);

    //         if (stopMiddleLow == false){
    //             lowerintake.move(lowerVelocity*-1.17);
    //             middle.move(midVelocity*-0.87);
    //         }
    //         else {
                
    //         } 
    // }
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){ // manual anti jam bc aryan is bum ig idrk
        
        scoring.move(scoringVelocity*-1.27*0.5);
        lowerintake.move(lowerVelocity*1.27*0.3);
        middle.move(midVelocity*1.27*0.3);

        
    }

    else {
        lowerintake.move(0);
        scoring.move(0);
        middle.move(0);
        buttonDone = false;
        tester = false;
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
    int LF = chassis.left_motors[5].get_temperature();
    int LM = chassis.left_motors[-6].get_temperature();
    int LB = chassis.left_motors[7].get_temperature();
    // right sides
    int RF = chassis.right_motors[8].get_temperature();
    int RM = chassis.right_motors[-9].get_temperature();
    int RB = chassis.right_motors[10].get_temperature();

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
        lowerintake.move(speed*-1);
    }
}

void turnScoringatXSpeed(bool state, int speed){
    if (state){
        scoring.move(speed*-1);
    }
}

void turnMiddleatXSpeed(bool state, int speed){
    if (state){
        middle.move(speed*-1);
    }
}

void antiJam(bool direction){// intake is true
    if (fabs(lowerintake.get_actual_velocity()) < 0.5 && fabs(lowerintake.get_voltage()) > 2000) {
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 20 && direction) {
            master.print(0,0, "This happens");
            master.rumble("-");
            lowerintake.move(0);
            intakeStuckTime = 0;
        }
        else if (pros::millis() - intakeStuckTime > 20 && direction==false) {
            master.print(0,0, "This happens");
            master.rumble("-");
            lowerintake.move(0);
            intakeStuckTime = 0;
        }
    }

    if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000) { // checks if scoring rollers are jammed
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200 && direction) {
            master.print(0,0, "This happens");
            master.rumble("-");
            scoring.move(0);
            intakeStuckTime = 0;
        }

        else if (pros::millis() - intakeStuckTime > 200 && direction == false) {
            master.print(0,0, "This happens");
            master.rumble("-");
            scoring.move(0);
            intakeStuckTime = 0;
        }
        
    }

}

void scoringJam(){
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000) { // checks if scoring rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "slow scoring");
            master.rumble("-");
            scoring.move(scoringVelocity*-0.1);

            stopScoring = true;
            intakeStuckTime = 0;
        }
    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && fabs(middle.get_actual_velocity()) < 0.5 && fabs(middle.get_voltage()) > 2000) { // checks if scoring rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "hahhhahha");
            master.rumble("-");
            middle.move(0);
            scoring.move(0);

            stopScoring = true;
            stopMiddle = true;
            intakeStuckTime = 0;
        }
    }

    
    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000){
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "testing new scoring");
            master.rumble("-");

            middle.move(100);
            scoring.move(-100);

            pros::delay(400);

            middle.move(-100);
            scoring.move(100);

            intakeStuckTime = 0;
        }




    }

    else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && fabs(lowerintake.get_actual_velocity()) < 0.5 && fabs(lowerintake.get_voltage()) > 2000){
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "testing new scoring");
            master.rumble("-");

            lowerintake.move(127);

            pros::delay(100);

            lowerintake.move(-127);

            intakeStuckTime = 0; 
        }




    }
    else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && stopScoring == true) {
        stopScoring = false;
    }
    else if (!master.get_digital(pros::E_CONTROLLER_DIGITAL_R2) && stopMiddle == true) {
        stopMiddle = false;
    }

}


void midJam(){ // istn called
    if (fabs(middle.get_actual_velocity()) < 0.5 && fabs(middle.get_voltage()) > 2000) { // checks if scoring rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "stop scoring");
            master.rumble("-");
            middle.move(0);

            stopMiddle = true;
            intakeStuckTime = 0;
        }
    }
}

void middleScoreJam(){ // isnt called
    
    if (fabs(middle.get_actual_velocity()) < 0.5 && fabs(middle.get_voltage()) > 2000) { // checks if scoring rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "stop scoring");
            master.rumble("-");
            middle.move(0);
            
        }
    }
}

void setAntiJam(bool state, bool antiS){
    antiJamOn = state;
    antiState = antiS;
}

void setAntiJamActiveWorlds(bool state){
    AntiJamActiveWorlds = state;
}

void autonAntiJam(){
    if (fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000) { // checks if scoring rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "slow scoring");
            master.rumble("-");
            scoring.move(scoringVelocity*-0.2);

            stopScoring = true;
            intakeStuckTime = 0;
        }
    }

    else if (fabs(middle.get_actual_velocity()) < 0.5 && fabs(middle.get_voltage()) > 2000) { // checks if middle rollers are jammed
    //     master.print(0,0, "start scoring");

        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "hahhhahha");
            master.rumble("-");
            middle.move(0);
            scoring.move(0);

            stopScoring = true;
            stopMiddle = true;
            intakeStuckTime = 0;
        }
    }

    
    else if (fabs(scoring.get_actual_velocity()) < 0.5 && fabs(scoring.get_voltage()) > 2000){
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "testing new scoring");
            master.rumble("-");

            middle.move(100);
            scoring.move(-100);

            pros::delay(400);

            middle.move(-100);
            scoring.move(100);
            intakeStuckTime = 0;
        }




    }

    else if (fabs(lowerintake.get_actual_velocity()) < 0.5 && fabs(lowerintake.get_voltage()) > 2000){
        if (intakeStuckTime == 0) {
            intakeStuckTime = pros::millis();
        }
        else if (pros::millis() - intakeStuckTime > 200) {
            master.print(0,0, "testing new scoring");
            master.rumble("-");

            lowerintake.move(127);

            pros::delay(300);

            lowerintake.move(-127);
            intakeStuckTime = 0;
        }




    }

}

void intakeTask(void* parameter) {
    lowerintake.tare_position();
    scoring.tare_position();

    
    while (true) {
        
            
        
        if (!pros::competition::is_autonomous()) { // if the intake is not in the autonomous state ex: op control

            lowerintake.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            scoring.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            intakeOpControl();
            


            scoringJam();
            
        
            
        }
        else if (pros::competition::is_autonomous()) {
            //heatSensingControl();
            // if(lowerintake.get_actual_velocity() > 0 and match){
                
            // }
            // if(antiJamOn){
            //     if(antiState){
            //         antiJam(true);
            //     }
            //     else{
            //         antiJam(false);
            //     }
            // }

            

            // master.clear();

            // pros::delay(50);

            // master.print(0,0,"Front: %i",frontD.get());

            // pros::delay(100);

            if(AntiJamActiveWorlds){
                autonAntiJam();

            }


        }

        pros::delay(20);
    }
}








