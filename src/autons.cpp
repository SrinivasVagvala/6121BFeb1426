#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "subsystems.hpp"
#include "usr/globals.hpp"
#include "usr/intake.hpp"
#include "usr/descore.hpp"
#include "usr/matchload.hpp"
#include "usr/middlegoal.hpp"
#include "subsystems.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include <iterator>

extern pros::Controller master;

extern pros::Distance frontD;
/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;


extern bool killSwitchOn;

void macroTask(void* parameter) {
  while (!pros::competition::is_autonomous()) {
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
      killSwitchOn = true;
    }
  }

  while (pros::competition::is_autonomous()) {
    //master.print(0,0,"Hue   %i",distanceSens.get());
  }
}

void default_constants() {
  // P, I, D, and Start I

  // kP 20 kD 100 for driving
  // turn in place constants are kP 4 and kD 23.0 and kI 0.05, then 15.0
  chassis.pid_drive_constants_set(16.5, 0.0, 153);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(15.5, 0.0, 24.5);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.8, 0.05, 22.0, 15.0);     // Turn in place constants
  chassis.pid_swing_constants_set(6.0, 0.0, 65.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(6.5, 0.0, 52.5);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.8, 0.0, 32.5);  // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(100_ms, 2_deg, 245_ms, 6_deg, 400_ms, 400_ms);
  chassis.pid_swing_exit_condition_set(100_ms, 3_deg, 260_ms, 7_deg, 510_ms, 510_ms);
  chassis.pid_drive_exit_condition_set(160_ms, 1_in, 305_ms, 3_in, 550_ms, 550_ms);
  chassis.pid_odom_turn_exit_condition_set(100_ms, 3_deg, 260_ms, 7_deg, 510_ms, 760_ms);
  chassis.pid_odom_drive_exit_condition_set(100_ms, 1_in, 260_ms, 3_in, 510_ms, 760_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(13_in, 540); // 12_in, 540
  chassis.slew_swing_constants_set(3_in, 80);

  // The amount that turns are prioritized over driving in odom motions
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

///
// Drive Example
///
void drive_example() {
  // The first parameter is target inches
  // The second parameter is max speed the robot will drive at
  // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
  // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

  //chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait();
}

void tuningDrivePID() {

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  
}

void tuningTurnPID() {
  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait();

}



///
// Turn Example
///
void turn_example() {
  // The first parameter is the target in degrees
  // The second parameter is max speed the robot will drive at

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
  // pid_wait_until will wait until the robot gets to a desired position

  // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(24_in, 30, true);
  chassis.pid_wait_until(6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
  chassis.pid_drive_set(-24_in, 30, true);
  chassis.pid_wait_until(-6_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);  // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
  chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
  // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
  // The second parameter is the target in degrees
  // The third parameter is the speed of the moving side of the drive
  // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

  chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
  chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
  // Motion chaining is where motions all try to blend together instead of individual movements.
  // This works by exiting while the robot is still moving a little bit.
  // To use this, replace pid_wait with pid_wait_quick_chain.
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-45_deg, TURN_SPEED);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  // Your final motion should still be a normal pid_wait
  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

///
// Interference example
///
void tug(int attempts) {
  for (int i = 0; i < attempts - 1; i++) {
    // Attempt to drive backward
    printf("i - %i", i);
    chassis.pid_drive_set(-12_in, 127);
    chassis.pid_wait();

    // If failsafed...
    if (chassis.interfered) {
      chassis.drive_sensor_reset();
      chassis.pid_drive_set(-2_in, 20);
      pros::delay(1000);
    }
    // If the robot successfully drove back, return
    else {
      return;
    }
  }
}

// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  if (chassis.interfered) {
    tug(3);
    return;
  }

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
  // This works the same as pid_drive_set, but it uses odom instead!
  // You can replace pid_drive_set with pid_odom_set and your robot will
  // have better error correction.

  chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_odom_set(-12_in, DRIVE_SPEED);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
  // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
  chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
                        {{0_in, 20_in}, fwd, DRIVE_SPEED},
                        {{0_in, 30_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  // Drive to 0, 0 backwards
  chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
  chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait_until_index(1);  // Waits until the robot passes 12, 24
  // Intake.move(127);  // Set your intake to start moving once it passes through the second point in the index
  chassis.pid_wait();
  // Intake.move(0);  // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
  chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
  chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
                        {{12_in, 24_in}, fwd, DRIVE_SPEED},
                        {{24_in, 24_in}, fwd, DRIVE_SPEED}},
                       true);
  chassis.pid_wait();

  chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
                       true);
  chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
  // Number of times to test
  int iterations = 10;

  // Our final offsets
  double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

  // Reset all trackers if they exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
  
  for (int i = 0; i < iterations; i++) {
    // Reset pid targets and get ready for running an auton
    chassis.pid_targets_reset();
    chassis.drive_imu_reset();
    chassis.drive_sensor_reset();
    //chassis.drive_brake_set(MOTOR_BRAKE_HOLD);
    chassis.odom_xyt_set(0_in, 0_in, 0_deg);
    double imu_start = chassis.odom_theta_get();
    double target = i % 2 == 0 ? 90 : 270;  // Switch the turn target every run from 270 to 90

    // Turn to target at half power
    chassis.pid_turn_set(target, 63, ez::raw);
    chassis.pid_wait();
    pros::delay(250);

    // Calculate delta in angle
    double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

    // Calculate delta in sensor values that exist
    double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
    double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
    double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
    double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

    // Calculate the radius that the robot traveled
    l_offset += l_delta / t_delta;
    r_offset += r_delta / t_delta;
    b_offset += b_delta / t_delta;
    f_offset += f_delta / t_delta;
  }

  // Average all offsets
  l_offset /= iterations;
  r_offset /= iterations;
  b_offset /= iterations;
  f_offset /= iterations;

  // Set new offsets to trackers that exist
  if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
  if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
  if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
  if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}

// . . .
// Make your own autonomous functions here!
// . . .

void testIntake(){
  matchLoadSorting();
}

void testPID(){
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();


  

  
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(48_in, DRIVE_SPEED*0.8, true);
  // chassis.pid_wait();

  // pros::delay(150);

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(48_in, DRIVE_SPEED*0.8, true);
  // chassis.pid_wait();

  // pros::delay(150);

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);



  // chassis.pid_drive_set(50_in, DRIVE_SPEED, true);
  // chassis.pid_wait();


  // chassis.pid_turn_set(180_deg, TURN_SPEED, false);
  // chassis.pid_wait();

  ///EXAMPLE CODE DOESN'T ACTUAL DO ANYTHING
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  // chassis.pid_drive_set(40_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // setStore(true);
  // chassis.pid_drive_set(-15_in, DRIVE_SPEED, true);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  // chassis.pid_turn_set(-140_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // setStore(false);
  ///END OF EXAMPLE CODE, DELETE ONCE YOUR DOT
  // setTeam(false);
  // setMatch(true);
  // intake(true);


}

// void DistanceReset(float distance, int speed){
//   chassis.drive_set(speed,speed);
//   while (distanceSens.get() > distance * 25.4 + 330){ //convert inches to mm, Distance only reads in mm in pros
//     pros::delay(1);
//   }
//   chassis.drive_set(0,0);
// }

// void DistanceResetTest(){
//   DistanceReset(20,DRIVE_SPEED*0.4);
// }

//AUTONS FOR PUSHBACK

void Inverse12C(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(26_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(-115_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(-46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true);

  chassis.pid_drive_set(-30.5_in, DRIVE_SPEED, true);
  pros::delay(650);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  matchloaderActive(true);

  scoreHigh(true); // scores in the high goal
  pros::delay(900); 

  reverseScoring(true);
  intake(true);

  setAntiJam(false);

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(745);

  chassis.pid_drive_set(-9_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(49_deg, TURN_SPEED, false); // turn to matchloader
  pros::delay(400);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-54_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(1050);

  middleGoalPiston(false);
  scoreMid(false, false);

  
  chassis.pid_drive_set(7_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();

  descoreMidToggle();

  chassis.pid_drive_set(-7_in, DRIVE_SPEED*0.7, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();
}

void Inverse3Plus4(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(26_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(-115_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(-46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true);

  chassis.pid_drive_set(-30.5_in, DRIVE_SPEED, true);
  pros::delay(625);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  matchloaderActive(true);


  scoreHigh(true); // scores in the high goal
  pros::delay(990); 

  reverseScoring(true);
  intake(true);

  setAntiJam(false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(16_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(835);

  chassis.pid_drive_set(-9_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(49_deg, TURN_SPEED, false); // turn to matchloader
  pros::delay(400);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-54_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(1050);

  middleGoalPiston(false);
  scoreMid(false, false);

  
  chassis.pid_drive_set(7_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();

  descoreMidToggle();

  chassis.pid_drive_set(-7_in, DRIVE_SPEED*0.45, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(39_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(-26.5_in, DRIVE_SPEED*0.7, true);
  chassis.pid_wait_quick_chain();

  descoreMidToggle();



 

}

void Left9BallAuton(){
   //setMatch(true); // Inits for auton instead of skills
  //setTeam(true); // Sets team to Red
  setAntiJam(true);
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often

  // chassis.pid_turn_set(-15_deg, TURN_SPEED, false); // turns to go triple blocks
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(35.5_in, DRIVE_SPEED*0.57, true);  // moves towards triple balls
  chassis.pid_wait();

  chassis.pid_turn_set(-41_deg, TURN_SPEED, false); // turn to two balls underneath long goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17.5_in, DRIVE_SPEED*0.60, true); // drives under the long goal to get the two underneath
  chassis.pid_wait();
  pros::delay(50); // Waiting for the block to intake from under the goal
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED, true); //go out from under long goal
  chassis.pid_wait();

  chassis.pid_turn_set(-83_deg, TURN_SPEED, false); // turns towards middle of goal/matchload area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(38.5_in, DRIVE_SPEED, true); // drive towards middle of goal/matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(-31_deg, TURN_SPEED, false); // turns towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED*0.75, true); //drive towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true);
  scoreHigh(true); // scores in the high goal
  matchloaderActive(true);
  pros::delay(1300); // Waiting for it to score
  setAntiJam(false);

  turnOffScoring(true);
  turnLoweratXSpeed(true,110);

  chassis.pid_drive_set(30_in, DRIVE_SPEED*0.9, true); //drive towards loader
  chassis.pid_wait();
  pros::delay(48); // unloads matchloader

  chassis.pid_drive_set(-30_in, DRIVE_SPEED*0.8, true); //moves toward the goal to do final score
  chassis.pid_wait();

  setAntiJam(true);
  scoreHigh(true); // scores in the high goal for second time
  pros::delay(700); 
  setAntiJam(false);


}

void Right9BallAuton(){
  //setMatch(true); // Inits for auton instead of skills
  //setTeam(true); // Sets team to Red
  setAntiJam(true);
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often

  // chassis.pid_turn_set(-15_deg, TURN_SPEED, false); // turns to go triple blocks
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(35.5_in, DRIVE_SPEED*0.57, true);  // moves towards triple balls
  chassis.pid_wait();

  chassis.pid_turn_set(41_deg, TURN_SPEED, false); // turn to two balls underneath long goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17.5_in, DRIVE_SPEED*0.60, true); // drives under the long goal to get the two underneath
  chassis.pid_wait();
  pros::delay(50); // Waiting for the block to intake from under the goal
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-16.5_in, DRIVE_SPEED, true); //go out from under long goal
  chassis.pid_wait();

  chassis.pid_turn_set(83_deg, TURN_SPEED, false); // turns towards middle of goal/matchload area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(38.5_in, DRIVE_SPEED, true); // drive towards middle of goal/matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(31_deg, TURN_SPEED, false); // turns towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED*0.75, true); //drive towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true);
  scoreHigh(true); // scores in the high goal
  matchloaderActive(true);
  pros::delay(1300); // Waiting for it to score
  setAntiJam(false);

  turnOffScoring(true);
  turnLoweratXSpeed(true,110);

  chassis.pid_drive_set(30_in, DRIVE_SPEED*0.9, true); //drive towards loader
  chassis.pid_wait();
  pros::delay(48); // unloads matchloader

  chassis.pid_drive_set(-30_in, DRIVE_SPEED*0.8, true); //moves toward the goal to do final score
  chassis.pid_wait();

  setAntiJam(true);
  scoreHigh(true); // scores in the high goal for second time
  pros::delay(700); 
  setAntiJam(false);


}

void Left2Plus7(){
  setMatch(false);
  setAntiJam(true);
  intake(true); // turns on intake
  MiddleAlignerActive(true);// turn on middle goal aligner

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often

  // chassis.pid_turn_set(-15_deg, TURN_SPEED, false); // turns to go triple blocks
  // chassis.pid_wait(); REMOVED SINCE CHANGED ALIGNMENT
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  

  chassis.pid_drive_set(35.5_in, DRIVE_SPEED*0.57, true);  // moves towards triple balls
  chassis.pid_wait();

  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED, true);  // align with goal
  chassis.pid_wait();

  chassis.pid_turn_set(-117_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-18_in, DRIVE_SPEED, true);  // moves towards middle goal
  chassis.pid_wait();

  scoreMid(true, false);
  pros::delay(250);
  scoreMid(false, false);
  pros::delay(5);
  intake(true);

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);  // moves away middle goal
  chassis.pid_wait();

  chassis.pid_turn_set(72_deg, TURN_SPEED, false); // turn to two balls
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(24_in, DRIVE_SPEED*0.93, true);  // moves to pick up two balls
  chassis.pid_wait();

  pros::delay(100);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true); //go out from under long goal
  chassis.pid_wait();

  chassis.pid_turn_set(-85_deg, TURN_SPEED, false); // turns towards middle of goal/matchload area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(39.5_in, DRIVE_SPEED, true); // drive towards middle of goal/matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(-30_deg, TURN_SPEED, false); // turns towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(false);

  chassis.pid_drive_set(-25_in, DRIVE_SPEED*1.2, true); //drive towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true); // prepare for matchload
  turnLoweratXSpeed(true,80);

  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.90, true); //drive towards matchloader
  chassis.pid_wait();

  pros::delay(10); // unloads matchloader

  chassis.pid_drive_set(-30_in, DRIVE_SPEED*1.2, true); //moves toward the goal to do final score
  chassis.pid_wait();

  extake(true);
  pros::delay(50);
  turnLoweratXSpeed(true,127);
  turnScoringatXSpeed(true,-127);
  pros::delay(900); 
  setAntiJam(false);

}

void Right7BallDescore(){
  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(33.5_in, DRIVE_SPEED*0.9, true);  // moves towards triple balls
  chassis.pid_wait_until(18_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(120_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(33.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();


  chassis.pid_turn_set(46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true); // prepare for matchload
  reverseScoring(false);
  setAntiJam(true);


  chassis.pid_drive_set(20_in, DRIVE_SPEED*0.65, true); // drive towards matchloader
  pros::delay(950);

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(820);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1500); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false);

  // descore starting on long goal

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-11_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-41_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait_quick_chain();


}

void Right4BallRush(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(27_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(115_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(36.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true);

  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
  pros::delay(755);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  scoreHigh(true); // scores in the high goal
  pros::delay(850); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false);

  // descore starting on long goal

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(42_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-40_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait_quick_chain();

}

void Right7BallPush(){
  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(33.5_in, DRIVE_SPEED*0.9, true);  // moves towards triple balls
  chassis.pid_wait_until(18_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(120_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();


  chassis.pid_turn_set(44_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true); // prepare for matchload
  reverseScoring(false);
  setAntiJam(true);


  chassis.pid_drive_set(20_in, DRIVE_SPEED*0.65, true); // drive towards matchloader
  pros::delay(950);

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(820);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1300); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false);

  // descore starting on long goal

  chassis.pid_drive_set(7_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();




} 

void Left7BallDescore(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(19_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(-115_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(35_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();
  
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true); // prepare for matchload
  reverseScoring(false);
  setAntiJam(true);

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(22_in, DRIVE_SPEED*0.55, true); // drive towards matchloader
  pros::delay(850);

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(775);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1250); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false);

  // descore starting on long goal

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(42_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-12.5_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-40_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(-21_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait_quick_chain();


}

void Left4BallRush(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(-115_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(36.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true);

  chassis.pid_drive_set(-30_in, DRIVE_SPEED, true);
  pros::delay(755);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  scoreHigh(true); // scores in the high goal
  pros::delay(850); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false);

  // descore starting on long goal

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(42_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-13_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-40_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(-21_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait_quick_chain();

}

void Left7BallPush(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(25_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(19_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(-115_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();
  
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true); // prepare for matchload
  reverseScoring(false);
  setAntiJam(true);

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(22_in, DRIVE_SPEED*0.55, true); // drive towards matchloader
  pros::delay(850);

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(775);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1250); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false);

  // push starting on long goal

  chassis.pid_drive_set(7_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

} 

void Left3Plus4(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake
  reverseScoring(true);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED, true);  // align with goal
  chassis.pid_wait();

  chassis.pid_turn_set(-116_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  pros::delay(820);

  turnLoweratXSpeed(true, 113);

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(800);

  middleGoalPiston(false);
  scoreMid(false, false);

  pros::delay(20);

  reverseScoring(true);
  intake(true);

  chassis.pid_drive_set(51.5_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  matchloaderActive(true); // prepare for matchload

  reverseScoring(false);

  chassis.pid_drive_set(-2_in, DRIVE_SPEED*0.65, true); // drive towards matchloader
  chassis.pid_wait();


  chassis.pid_drive_set(16.5_in, DRIVE_SPEED*0.45, true); // drive towards matchloader
  pros::delay(1000); // unloads matchloader

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(775);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1050); 

  turnOffScoring(true);
  intake(false);

  //descore starting on long goal

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-14.25_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-41_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(-21_in, DRIVE_SPEED*0.7, true); // descore balls in goal
  chassis.pid_wait_quick_chain();



}

void Left3Plus4Push(){
  setMatch(false);
  setAntiJam(false);
  wingToggle();
  intake(true); // turns on intake
  reverseScoring(true);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(35.5_in, DRIVE_SPEED*0.80, true);  // moves towards triple balls
  chassis.pid_wait_until(18_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);  // align with goal
  chassis.pid_wait();

  chassis.pid_turn_set(-116_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-19.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  pros::delay(800);

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(1000);

  pros::delay(20);

  reverseScoring(true);

  chassis.pid_drive_set(52.5_in, DRIVE_SPEED*0.9, true);  // moves to matchload area
  chassis.pid_wait();

  middleGoalPiston(false);
  scoreMid(false, false);
  intake(true);

  chassis.pid_turn_set(-46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  matchloaderActive(true); // prepare for matchload

  chassis.pid_drive_set(-2_in, DRIVE_SPEED*0.65, true); // drive towards matchloader
  chassis.pid_wait();


  chassis.pid_drive_set(16_in, DRIVE_SPEED*0.65, true); // drive towards matchloader
  chassis.pid_wait();

  reverseScoring(false);

  turnLoweratXSpeed(true,100);

  pros::delay(10); // unloads matchloader

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(775);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1000); 

  turnOffScoring(true);
  intake(false);

  //push starting on long goal

  chassis.pid_drive_set(10_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait();


  pros::delay(100);

  
  chassis.pid_drive_set(-14_in, 127, true); // drive out of goal
  pros::delay(1500);
}

void Right4Plus6(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);
  setMatch(false);
  setAntiJam(false);

  wingToggle();
  intake(true); // turns on intake


  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  // first long goal scoring

  chassis.pid_drive_set(32_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();
  
  intake(true);

  chassis.pid_turn_set(91_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(11.75_in,DRIVE_SPEED*0.65,true); // move into first matchloader to empty
  pros::delay(870); // manual timeout for speed

  chassis.pid_drive_set(-32_in,DRIVE_SPEED,true); // align with goal-check
  pros::delay(760);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);
  scoreHigh(true);  // score for the first time
  pros::delay(830);

  turnOffScoring(true);

  intake(true);
  
  chassis.pid_drive_set(17_in, DRIVE_SPEED, true);
  pros::delay(570);

  chassis.pid_turn_set(130_deg, TURN_SPEED, false); // turn to face 1st quartet
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); // drive into quartet
  chassis.pid_wait_until(20_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(50);
  
  matchloaderInactive(true);


  

  chassis.pid_turn_set(-40_deg, TURN_SPEED, false); // turn to face other quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(49.5_in, DRIVE_SPEED, true); // drive into other quartet
  chassis.pid_wait_until(31_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-3_in);
  matchloaderInactive(true);
  chassis.pid_wait();


  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to face middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(-19_in, DRIVE_SPEED*0.75, true); // drive into middle goal
  pros::delay(570);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(50);
  setAntiJam(true);

  middleGoalPiston(true);
  scoreMid(true, true); // score into middle goal

  setAntiJam(false);
}
       
void Left8BallAuton(){
  //setMatch(true); // Inits for auton instead of skills
  //setTeam(true); // Sets team to Red
  intake(true); // turns on intake


  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  chassis.pid_turn_set(-15_deg, TURN_SPEED, false); // turns to go triple blocks
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
 
  chassis.pid_drive_set(40.5_in, DRIVE_SPEED*0.35, true); // moves towards triple balls slowly at first and then speeds up
  chassis.pid_wait_until(20.5); 
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(-63_deg, TURN_SPEED, false); // turn directly at long goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(15_in, DRIVE_SPEED*0.70, true); // drives under the long goal to get the two underneath
  chassis.pid_wait();
  pros::delay(100); // Waiting for the block to intake from under the goal

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true); // go out from under long goal
  chassis.pid_wait();


  chassis.pid_turn_set(-55_deg, TURN_SPEED, false);  // turns towards middle of goal/matchload area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(43_in, DRIVE_SPEED, true); // drive towards middle of goal/matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(-38_deg, TURN_SPEED, false); // turns towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-19_in, DRIVE_SPEED*0.8, true); // drive towards goal
  pros::delay(375);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true);
  scoreHigh(true); // scores in the high goal
  matchloaderActive(true);
  chassis.pid_wait();
  pros::delay(350); 
  setAntiJam(false);

  turnOffScoring(true);
  turnLoweratXSpeed(true,110);

  chassis.pid_wait();
  chassis.pid_drive_set(31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(26); // unloads matchloader


  chassis.pid_drive_set(-31_in, DRIVE_SPEED*1, true); //moves toward the goal to do final score
  chassis.pid_wait();


  scoreHigh(true); // scores in the high goal
  pros::delay(900); // Waiting for it to score

 }

void Right8BallAuton(){
    //setMatch(true); // Inits for auton instead of skills
  //setTeam(true); // Sets team to Red
  intake(true); // turns on intake


  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  chassis.pid_turn_set(15_deg, TURN_SPEED, false); // turns to go triple blocks
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
 
  chassis.pid_drive_set(40.5_in, DRIVE_SPEED*0.35, true); // moves towards triple balls slowly at first and then speeds up
  chassis.pid_wait_until(20.5); 
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(63_deg, TURN_SPEED, false); // turn directly at long goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(15_in, DRIVE_SPEED*0.70, true); // drives under the long goal to get the two underneath
  chassis.pid_wait();
  pros::delay(100); // Waiting for the block to intake from under the goal

  chassis.pid_drive_set(-20_in, DRIVE_SPEED, true); // go out from under long goal
  chassis.pid_wait();


  chassis.pid_turn_set(55_deg, TURN_SPEED, false);  // turns towards middle of goal/matchload area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(41_in, DRIVE_SPEED, true); // drive towards middle of goal/matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(38_deg, TURN_SPEED, false); // turns towards goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-19_in, DRIVE_SPEED*0.8, true); // drive towards goal
  pros::delay(375);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true);
  scoreHigh(true); // scores in the high goal
  matchloaderActive(true);
  chassis.pid_wait();
  pros::delay(350); 
  setAntiJam(false);

  turnOffScoring(true);
  turnLoweratXSpeed(true,110);

  chassis.pid_wait();
  chassis.pid_drive_set(31_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  pros::delay(26); // unloads matchloader


  chassis.pid_drive_set(-31_in, DRIVE_SPEED*1, true); //moves toward the goal to do final score
  chassis.pid_wait();


  scoreHigh(true); // scores in the high goal
  pros::delay(900); // Waiting for it to score

}

void progSkills81(){
  setMatch(false); // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false);

  wingToggle();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  matchloaderActive(true);
  intake(true);
  
  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.75,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(11.5_in,DRIVE_SPEED*0.6,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(650); // first matchloader delay

  chassis.pid_drive_set(-32.75_in,DRIVE_SPEED*0.8,true); // align with goal-check
  pros::delay(890);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  pros::delay(50);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_turn_set(47_deg,TURN_SPEED*0.86,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  turnLoweratXSpeed(true, 30);
  
  chassis.pid_drive_set(14.5_in,DRIVE_SPEED,true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(-49_deg,TURN_SPEED*0.9,false); // turn into alley, aligned to drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-75_in,DRIVE_SPEED*0.90,true); // drive across alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_set(13_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-10_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to move back more
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-14_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(125_deg, TURN_SPEED, false); // align to score on goal for first time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in,DRIVE_SPEED,true); // drive into goal
  pros::delay(705);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  // extake(true);
  // pros::delay(150);
  setAntiJam(true);
  scoreHigh(true);  // score for the first time
  pros::delay(1900);
  setAntiJam(false);

  turnOffScoring(true);

  pros::delay(15);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(36_in,DRIVE_SPEED*0.45,true); // drive into 2nd matchloader
  chassis.pid_wait();

  pros::delay(675); // second matchloader delay

  chassis.pid_drive_set(-32_in,DRIVE_SPEED*0.75,true); // drive into goal
  pros::delay(880);
  //chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  setAntiJam(true);
  scoreHigh(true); // score into goal second time
  pros::delay(1800);
  turnOffScoring(true);
  intake(false);
  setAntiJam(false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // middle goal scoring


  chassis.pid_drive_set(12_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  intake(true);

  chassis.pid_turn_set(123_deg, TURN_SPEED, false); // turn to face 1st quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32.5_in, DRIVE_SPEED*0.80, true); // drive into quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  

  chassis.pid_drive_set(-3_in, DRIVE_SPEED*0.85, true); // align with other quartet
  chassis.pid_wait();

  //reverseScoring(true);
  intake(true);

  chassis.pid_turn_set(-35_deg, TURN_SPEED, false); // turn to face other quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(51.5_in, DRIVE_SPEED*0.95, true); // drive into other quartet
  chassis.pid_wait_until(18_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.32);
  scoreHigh(true);
  pros::delay(170);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait_until(43_in);
  //matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  intake(false);


  chassis.pid_drive_set(-5.5_in, DRIVE_SPEED*0.82, true); // move back to get ready for middle goal
  chassis.pid_wait_until(3_in);
  matchloaderInactive(true);
  chassis.pid_wait();

  chassis.pid_turn_set(-43_deg, TURN_SPEED, false); // turn to face middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED*0.57, true); // drive into middle goal
  chassis.pid_wait();
  
  extake(true);
  pros::delay(100);
  setAntiJam(true);

  middleGoalPiston(true);
  scoreMid(true, false); // score into middle goal

  setAntiJam(false);

  pros::delay(1900);

  

  matchloaderInactive(true);
  turnOffScoring(true);
  intake(true);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(53.5_in, DRIVE_SPEED*0.9, true); // drive into goal/matchload area
  chassis.pid_wait();

  middleGoalPiston(false);
  scoreMid(false, false);

 if(frontD.get() > 670 || frontD.get() < 663){
    float DistanceToMove = frontD.get() - 667;
    DistanceToMove *= 0.03937008;
    chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
    chassis.pid_wait();
  }


  chassis.pid_turn_set(-46.5_deg, TURN_SPEED, false); // turn to face long goal for align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-24_in,DRIVE_SPEED*0.6,true); // align with goal
  pros::delay(1200);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);
  matchloaderActive(true);


  // end middle goal



  chassis.pid_drive_set(31.5_in,DRIVE_SPEED*0.52,true); // third matchloader
  chassis.pid_wait();

  pros::delay(600); // third matchloader delay

  chassis.pid_drive_set(-31_in,DRIVE_SPEED*0.82,true); // align with goal for goal check
  pros::delay(1000);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal check
  chassis.pid_wait();

  turnLoweratXSpeed(true, 50);

  chassis.pid_turn_set(43_deg,TURN_SPEED,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(16_in,DRIVE_SPEED,true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(-44_deg,TURN_SPEED,false); // turn to be parallel to alley and drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-76_in,DRIVE_SPEED*0.90,true); // drive across alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_set(13_deg, TURN_SPEED, false); // turn to move out of alley 
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-11_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to realign
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-13.25_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(126_deg, TURN_SPEED, false); // turn to align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-23_in,DRIVE_SPEED,true); // drive into goal for 3rd score
  pros::delay(900);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  setAntiJam(true);
  scoreHigh(true); // score goal for the third time
  pros::delay(1700);
  setAntiJam(false);

  turnOffScoring(true);

  chassis.pid_drive_set(32_in,DRIVE_SPEED*0.55,true); // drive into 4th matchloader
  chassis.pid_wait();

  pros::delay(700);
  
  chassis.pid_drive_set(-33_in,DRIVE_SPEED*0.88,true); // drive into goal and for final score
  pros::delay(895);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  // extake(true); // score into goal for fourth time
  // pros::delay(200);
  setAntiJam(true);
  scoreHigh(true);
  pros::delay(1800);
  setAntiJam(false);
  turnOffScoring(true);

  // park

  
  intake(true); // turn on intake to prepare for park clear
  


  chassis.pid_drive_set(11.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.35);

  chassis.pid_odom_set({{27.5_in, 25.5_in, 90_deg}, fwd, 73});
  chassis.pid_wait();

  chassis.pid_drive_set(1.5_in, 75, true);
  chassis.pid_wait();

  pros::delay(25);


  chassis.pid_drive_set(31_in, 127, true);
  chassis.pid_wait();

  // master.clear();

  // pros::delay(50);

  // master.print(0,0,"Front: %i",frontD.get());
  
  // pros::delay(750);

  // if(frontD.get() > 2000){
    
  //   pros::delay(100);

  //   chassis.pid_drive_set(22_in, 127, true);
  //   chassis.pid_wait();
  // }

}

void progSkills96(){
  
  intake(true); // turn on intake to prepare for park clear
  


  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.35);

  chassis.pid_odom_set({{27.5_in, 26.5_in, 93_deg}, fwd, 73});
  chassis.pid_wait();

  chassis.pid_drive_set(1.5_in, 50, true);
  chassis.pid_wait();

  pros::delay(50);


  chassis.pid_drive_set(26_in, 127, true);
  chassis.pid_wait();

  //if (backD.get() - 104 > 1200){

  //   float distanceToMove = backD.get() - 104 - 1200;
  //   distanceToMove *= -0.03937008;

  //   chassis.pid_drive_set(distanceToMove, DRIVE_SPEED*0.2, true);
  //   chassis.pid_wait();
  // }


  // master.clear();

  // pros::delay(50);

  // master.print(0,0,"Front: %i",frontD.get() - 287);

  // pros::delay(50);

  // master.print(1,0,"Back: %i",backD.get() - 104);

  // pros::delay(1000);

  // if (backD.get() - 104 > 1200){

  //   float distanceToMove = backD.get() - 104 - 1200;
  //   distanceToMove *= -0.03937008;

  //   chassis.pid_drive_set(distanceToMove, DRIVE_SPEED*0.2, true);
  //   chassis.pid_wait();
  // }

  

  // chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.odom_boomerang_dlead_set(0.3);

  // chassis.pid_odom_set({{20_in, 30_in, 90_deg}, fwd, 70});
  // chassis.pid_wait();

  // if (backD.get() - 88 > 1200){

  //   float distanceToMove = backD.get()-88 - 1000;
  //   distanceToMove *= -0.03937008;

  //   chassis.pid_drive_set(distanceToMove, DRIVE_SPEED*0.2, true);
  //   chassis.pid_wait();
  // }

  // chassis.pid_drive_set(28_in, DRIVE_SPEED*0.5, true);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(43_in, DRIVE_SPEED*0.5, true);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // if (frontD.get() - 275 < 958){

  //   float distanceToMove = 958+275-frontD.get();
  //   distanceToMove *= -0.03937008;

  //   chassis.pid_drive_set(distanceToMove, DRIVE_SPEED*0.2, true);
  //   chassis.pid_wait();
  // }
}


  

void progSkills75(){
  setMatch(false); // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false);

  wingToggle();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  matchloaderActive(true);
  intake(true);
  
  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.75,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(11.5_in,DRIVE_SPEED*0.6,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(700); // first matchloader delay

  chassis.pid_drive_set(-32.75_in,DRIVE_SPEED*0.8,true); // align with goal-check
  pros::delay(890);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  pros::delay(50);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_turn_set(47_deg,TURN_SPEED*0.9,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  turnLoweratXSpeed(true, 30);
  
  chassis.pid_drive_set(14.5_in,DRIVE_SPEED,true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(-49_deg,TURN_SPEED*0.9,false); // turn into alley, aligned to drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-75_in,DRIVE_SPEED*0.8,true); // drive across alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_set(13_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-10_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to move back more
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-14_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(123_deg, TURN_SPEED, false); // align to score on goal for first time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in,DRIVE_SPEED,true); // drive into goal
  pros::delay(705);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  // extake(true);
  // pros::delay(150);
  setAntiJam(true);
  scoreHigh(true);  // score for the first time
  pros::delay(2000);
  setAntiJam(false);

  turnOffScoring(true);

  pros::delay(15);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(36_in,DRIVE_SPEED*0.45,true); // drive into 2nd matchloader
  chassis.pid_wait();

  pros::delay(650); // second matchloader delay

  chassis.pid_drive_set(-32_in,DRIVE_SPEED*0.75,true); // drive into goal
  pros::delay(880);
  //chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  setAntiJam(true);
  scoreHigh(true); // score into goal second time
  pros::delay(1800);
  turnOffScoring(true);
  intake(false);
  setAntiJam(false);

 

  chassis.pid_drive_set(-1_in,DRIVE_SPEED, false); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(15_in,DRIVE_SPEED*0.7,true); // drive away from goal
  chassis.pid_wait();

  chassis.pid_turn_set(89_deg, TURN_SPEED, false); //turn to other side
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(94.5_in,DRIVE_SPEED*0.7,true); // cross field
  chassis.pid_wait();

  chassis.pid_turn_set(-91_deg, TURN_SPEED, false); //turn to face goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  intake(true);

  chassis.pid_drive_set(-19.5_in,DRIVE_SPEED*0.7,true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(31.5_in,DRIVE_SPEED*0.52,true); // third matchloader
  chassis.pid_wait();

  pros::delay(600); // third matchloader delay

  chassis.pid_drive_set(-31_in,DRIVE_SPEED*0.82,true); // align with goal for goal check
  pros::delay(1000);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal check
  chassis.pid_wait();

  turnLoweratXSpeed(true, 50);

  chassis.pid_turn_set(43_deg,TURN_SPEED,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(16_in,DRIVE_SPEED,true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(-44_deg,TURN_SPEED,false); // turn to be parallel to alley and drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-76_in,DRIVE_SPEED*0.82,true); // drive across alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_set(13_deg, TURN_SPEED, false); // turn to move out of alley 
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-11_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to realign
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-14.5_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(125_deg, TURN_SPEED, false); // turn to align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-23_in,DRIVE_SPEED,true); // drive into goal for 3rd score
  pros::delay(900);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  setAntiJam(true);
  scoreHigh(true); // score goal for the third time
  pros::delay(1800);
  setAntiJam(false);

  turnOffScoring(true);

  chassis.pid_drive_set(32_in,DRIVE_SPEED*0.55,true); // drive into 4th matchloader
  chassis.pid_wait();

  pros::delay(700);
  
  chassis.pid_drive_set(-33_in,DRIVE_SPEED*0.88,true); // drive into goal and for final score
  pros::delay(895);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  // extake(true); // score into goal for fourth time
  // pros::delay(200);
  setAntiJam(true);
  scoreHigh(true);
  pros::delay(1800);
  setAntiJam(false);
  turnOffScoring(true);

  // park

  intake(true); // turn on intake to prepare for park clear
  


  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 73});
  chassis.pid_wait();


  chassis.pid_drive_set(48_in, 127, true);
  chassis.pid_wait();

}

void progSkills51(){
  setMatch(false);  // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false);

  chassis.pid_drive_set(31_in,DRIVE_SPEED*0.9,true); // drive off the park platform
  chassis.pid_wait();

  matchloaderActive(true);
  intake(true);

  pros::delay(200);

  chassis.pid_turn_set(86_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(6.25_in,DRIVE_SPEED*0.65,true); // move into matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(1200); // first matchloader delay

  chassis.pid_drive_set(-32_in, DRIVE_SPEED*0.7,true); // move back into goal to score for the first time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(100);
  scoreHigh(true); // score into goal 
  matchloaderInactive(true);
  pros::delay(3000);

  turnOffScoring(true);
  intake(true);

  chassis.pid_drive_set(13_in, DRIVE_SPEED*0.45, true);  // move away from goal
  chassis.pid_wait();

  chassis.pid_turn_set(120_deg,TURN_SPEED,false); // turn to go towards to the quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true);
  chassis.pid_drive_set(28_in, DRIVE_SPEED*0.3, true);  // drive to the quartet
  chassis.pid_wait();

  chassis.pid_turn_set(-29_deg,TURN_SPEED,false); // turn to face the blocks
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(24_in, DRIVE_SPEED*0.5, true);  // move to middle of the field
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg,TURN_SPEED,false); // turn avoid the second quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(25_in, DRIVE_SPEED*0.5, true);  // move away from the second quarter
  chassis.pid_wait();

  
  chassis.pid_turn_set(45_deg,TURN_SPEED,false); // turn to be perpendicular to the goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(28.5_in, DRIVE_SPEED*0.5, true);  // drive to prepare to score
  chassis.pid_wait();

  
  chassis.pid_turn_set(-90_deg,TURN_SPEED,false); // turn to score
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-21_in, DRIVE_SPEED*0.7, true);  // drive to goal 
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(50);
  scoreHigh(true); // score the quartet on the goal
  pros::delay(1500);

  turnOffScoring(true); // prepare to matchload
  turnLoweratXSpeed(true,127);
  matchloaderActive(true);
  setAntiJam(false);

  chassis.pid_drive_set(29.5_in, DRIVE_SPEED*0.48, true); // drives to matchload
  chassis.pid_wait();
  pros::delay(950); // unloads 2nd matchloader

  chassis.pid_drive_set(-34_in, DRIVE_SPEED*0.8, true);  //moves toward the goal to score second time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  matchloaderInactive(true);
  scoreHigh(true); // scores the second matchloader in the goal
  pros::delay(2400); 

  turnOffScoring(true);
  intake(true);

  chassis.pid_drive_set(13_in, DRIVE_SPEED*0.5, true);  // move away from goal
  chassis.pid_wait();

  chassis.pid_turn_set(-120_deg,TURN_SPEED,false); // turn to go towards to the quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true);
  chassis.pid_drive_set(28_in, DRIVE_SPEED*0.3, true);  // drive to the quartet
  chassis.pid_wait();

  pros::delay(1200); // delay to ensure pickup

  chassis.pid_drive_set(-31.5_in, DRIVE_SPEED*0.3, true);  // drive back from the quarter
  chassis.pid_wait();

  chassis.pid_turn_set(122_deg,TURN_SPEED,false); // turn to go back to the goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-17.5_in, DRIVE_SPEED*0.5, true);  // drive back to goal to score
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  scoreHigh(true); // score last quartet on the goal

  pros::delay(1500);

  setAntiJam(false);
  turnOffScoring(true);

  intake(true); // turn on intake to prepare for park clear

  chassis.pid_drive_set(15_in, DRIVE_SPEED*0.5, true); // move away from goal
  chassis.pid_wait();

  chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to face park 
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(24_in, DRIVE_SPEED, true); // move to the park 
  pros::delay(1000);

  chassis.drive_set(0,DRIVE_SPEED*0.5);
  pros::delay(400);
  chassis.drive_set(0,0);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(9_in, DRIVE_SPEED, false); // move to head on
  chassis.pid_wait_quick();

  pros::delay(100);

  matchloaderActive(true);  // drop matchload to help clear

  pros::delay(100);
  
  chassis.drive_set(0,DRIVE_SPEED*0.5);
  pros::delay(200);
  chassis.drive_set(0,0);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(100);

  chassis.pid_drive_set(44_in, 127, false); // clear park zone
  chassis.pid_wait_quick();

  pros::delay(100);

  matchloaderInactive(true); // put up matchloader to ensure that all balls are off the ground

}

void safeSkills(){
  chassis.pid_drive_set(-5_in, 127, true); // move back to build momentum
  chassis.pid_wait();
  
  intake(true); // turn on the intake to prepare for park clear

  matchloaderActive(true); // drop matchloader to ensure for park clear 

  pros::delay(200);

  chassis.pid_drive_set(55_in, 127, true); // drive to clear the park zone 
  chassis.pid_wait();

  matchloaderInactive(true);

}

void SOAP(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);
  setMatch(false);
  setAntiJam(false);

  wingToggle();
  intake(true); // turns on intake


  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  // first long goal scoring

  chassis.pid_drive_set(32_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();
  
  intake(true);

  chassis.pid_turn_set(91_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED,true);
  chassis.pid_wait_quick_chain();

  chassis.pid_drive_set(11.75_in,DRIVE_SPEED*0.65,true); // move into first matchloader to empty
  pros::delay(870); // manual timeout for speed

  chassis.pid_drive_set(-32_in,DRIVE_SPEED,true); // align with goal-check
  pros::delay(760);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);
  scoreHigh(true);  // score for the first time
  pros::delay(830);

  turnOffScoring(true);

  intake(true);
  
  chassis.pid_drive_set(17_in, DRIVE_SPEED, true);
  pros::delay(570);

  chassis.pid_turn_set(130_deg, TURN_SPEED, false); // turn to face 1st quartet
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true); // drive into quartet
  chassis.pid_wait_until(20_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(50);
  
  matchloaderInactive(true);


  

  chassis.pid_turn_set(-40_deg, TURN_SPEED, false); // turn to face other quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(49.5_in, DRIVE_SPEED, true); // drive into other quartet
  chassis.pid_wait_until(31_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-7_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-3_in);
  matchloaderInactive(true);
  chassis.pid_wait();


  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to face middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(-19_in, DRIVE_SPEED*0.75, true); // drive into middle goal
  pros::delay(570);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(50);
  setAntiJam(true);

  middleGoalPiston(true);
  scoreMid(true, true); // score into middle goal

  setAntiJam(false);

  pros::delay(700);

  middleGoalPiston(false);
  scoreMid(false, false);

  reverseScoring(true);
  intake(true);

  //end middle goal scoring

  chassis.pid_drive_set(53_in, DRIVE_SPEED, true); // lurch into goal/matchload area
  pros::delay(1200);

  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to face long goal for align
  pros::delay(500);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  

  chassis.pid_drive_set(-24_in,DRIVE_SPEED,true); // align with goal for goal check
  pros::delay(700);
  
  scoreHigh(true);
}

void MomAndDadAreFighting(){
  chassis.pid_turn_chain_constant_set(1_deg);
  chassis.pid_drive_chain_constant_set(2_in);
  setMatch(false);
  setAntiJam(false);

  wingToggle();
  // intake(true); // turns on intake


  // chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  // // first long goal scoring

  // chassis.pid_drive_set(33.5_in,DRIVE_SPEED*0.9,true); // drive off the park area
  // chassis.pid_wait();
  
  // intake(true);

  // chassis.pid_turn_set(92_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // matchloaderActive(true);

  // chassis.pid_drive_set(-5_in,DRIVE_SPEED,true);
  // chassis.pid_wait();

  // chassis.pid_drive_set(11_in,DRIVE_SPEED*0.9,true); // move into first matchloader to empty
  // pros::delay(800); // manual timeout for speed

  // chassis.pid_drive_set(-32_in,DRIVE_SPEED*0.8,true); // align with goal-check
  // pros::delay(800);
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);
  scoreHigh(true);  // score for the first time
  pros::delay(815);

  turnOffScoring(true);
  reverseScoring(true);

  intake(true);
  
  chassis.pid_drive_set(13_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(104_deg, TURN_SPEED, false); // turn to face 1st quartet
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(72_in, DRIVE_SPEED, true); // drive into quartet
  chassis.pid_wait_until(55_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(50);
  
  matchloaderInactive(true);

  chassis.pid_drive_set(-6.5_in, DRIVE_SPEED, true); // align to middle goal
  chassis.pid_wait();

  chassis.pid_turn_set(-63_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-16_in, DRIVE_SPEED, true); // drive into middle goal
  chassis.pid_wait();

  middleGoalPiston(true);
  scoreMid(true, true);
  turnLoweratXSpeed(true,88);

  pros::delay(850);

  middleGoalPiston(false);
  scoreMid(false, false);

  intake(true);

  chassis.pid_drive_set(57_in, DRIVE_SPEED, true); // drive into goal/matchload area
  chassis.pid_wait();

  chassis.pid_turn_set(-44_deg, TURN_SPEED, false); // turn to face long goal for align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  matchloaderActive(true);

  turnScoringatXSpeed(true, 30);

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true); // 2nd matchloader
  chassis.pid_wait();
  
  chassis.pid_drive_set(12.5_in, DRIVE_SPEED, true); // 2nd matchloader

  pros::delay(800);

  chassis.pid_drive_set(-31_in,DRIVE_SPEED,true); // align with goal for goal check
  chassis.pid_wait_until(28_in);
  scoreHigh(true);
  pros::delay(1050);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(1200);

  turnOffScoring(true);
  turnOffLower(true);

}
