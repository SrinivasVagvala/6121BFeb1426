#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/distance.hpp"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "usr/globals.hpp"
#include "usr/intake.hpp"
#include "usr/descore.hpp"
#include "usr/matchload.hpp"
#include "usr/middlegoal.hpp"
#include "subsystems.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "usr/quadsensor.hpp"

#include <cmath>
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
  chassis.pid_drive_constants_set(16, 0.0, 169);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(15.5, 0.0, 27);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(4.75, 0.05, 28.0, 10.0);     // Turn in place constants
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
}

void tuningDrivePID() {

  chassis.pid_drive_set(96_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  
  pros::delay(500);


  
  chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(500);

  chassis.pid_drive_set(-48_in, DRIVE_SPEED, true);
  chassis.pid_wait();

   pros::delay(500);

  chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
  chassis.pid_wait();



  
}

void tuningTurnPID() {

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_turn_set(90_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(500);

  chassis.pid_turn_set(180_deg, TURN_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  pros::delay(500);


  chassis.pid_turn_set(-90_deg, TURN_SPEED, true);
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

  // chassis.pid_turn_set(45_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_turn_set(0_deg, TURN_SPEED);
  // chassis.pid_wait();

  // chassis.pid_turn_set(180_deg, TURN_SPEED);
  // chassis.pid_wait();
}

///
// Combining Turn + Drive
///
void drive_and_turn() {
  chassis.pid_drive_set(96_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_turn_set(180_deg, TURN_SPEED, false);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(96_in, DRIVE_SPEED, true);
  chassis.pid_wait();
}

void square() {
  chassis.pid_drive_set(54_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(54_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(54_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(54_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED);
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
  setAntiJam(false, false);
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
  setAntiJam(true, true);

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

  setAntiJam(false, false);

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

  midDescoreToggle();

  chassis.pid_drive_set(-7_in, DRIVE_SPEED*0.7, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();
}

void Inverse3Plus4(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true);  // moves towards triple balls
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.75);
  chassis.pid_wait_until(14.5_in);
  matchloaderActive(true);
  pros::delay(50);
  chassis.pid_wait_until(27.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();



  chassis.pid_turn_set(-103_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(-56_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true, true);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED*0.9, true);
  pros::delay(200);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);


  scoreHigh(true); // scores in the high goal
  pros::delay(990); 

  

  setAntiJam(false, false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(16_in);
  reverseScoring(true);
  intake(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(775);

  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-53_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(650);

  middleGoalPiston(false);
  scoreMid(false, false);

  
  chassis.pid_drive_set(7_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();

  midDescoreToggle();

  chassis.pid_drive_set(-7.25_in, DRIVE_SPEED, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.drive_set(DRIVE_SPEED*0.85, DRIVE_SPEED*0.0);
  pros::delay(620);
  chassis.drive_set(0, 0);
  pros::delay(400);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  wingToggle();
  chassis.pid_drive_set(18_in, DRIVE_SPEED, true);
  chassis.pid_wait();




}

void LowInverse3Plus4(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true);  // moves towards triple balls
  chassis.pid_wait_until(10.5_in);
  matchloaderActive(true);
  chassis.pid_wait_until(27.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();

  

  chassis.pid_turn_set(105_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(59_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true, true);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-1_in);
  scoreHigh(true);
  pros::delay(200);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);



  pros::delay(920); 


  setAntiJam(false, false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32.2_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(16_in);
  intake(true);
  turnOffScoring(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(755);

  chassis.pid_drive_set(-9_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  chassis.pid_wait_until(-5.5_in);
  intake(false);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(48.5_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_until(33.5_in);
  scoreLow(true);
  chassis.pid_wait();

  pros::delay(400);

  intake(true);

  turnOffScoring(true);

  chassis.pid_drive_set(-31.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick();

  chassis.pid_turn_set(-138_deg, TURN_SPEED, false);
  chassis.pid_wait_quick_chain();

  wingToggle();

  chassis.pid_drive_set(-25_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

}

void Right7BallDescore(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true);  // moves towards triple balls
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.75);
  chassis.pid_wait_until(14.5_in);
  matchloaderActive(true);
  pros::delay(50);
  chassis.pid_wait_until(27.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();



  chassis.pid_turn_set(106_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(56.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true, true);

  
  chassis.pid_drive_set(28.25_in, DRIVE_SPEED*0.95, true); 
  chassis.pid_wait_until(10_in);
  reverseScoring(true);
  intake(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.2);
  pros::delay(755);

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  
  pros::delay(800);

  extake(true);
  pros::delay(120);
  scoreHigh(true);
  matchloaderInactive(true);
  pros::delay(1300); 

  turnOffScoring(true);
  intake(false);
  setAntiJam(false, false);

  // descore starting on long goal

  wingToggle();

  extake(true);

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-12.5_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(40_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  pros::delay(50);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait_quick_chain();

}

void Right4BallRush(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true);  // moves towards triple balls
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.75);
  chassis.pid_wait_until(14.5_in);
  matchloaderActive(true);
  pros::delay(50);
  chassis.pid_wait_until(27.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();



  chassis.pid_turn_set(106_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(55_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true, true);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED*0.9, true);
  chassis.pid_wait_until(-2_in);
  scoreHigh(true);
  pros::delay(180);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  matchloaderInactive(true);

  //scoreHigh(true); // scores in the high goal
  pros::delay(990); 


  setAntiJam(false, false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // descore starting on long goal


  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  wingToggle();


  chassis.pid_turn_set(-43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-9_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(39_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  pros::delay(50);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait();

}

void Left7BallDescore(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true);  // moves towards triple balls
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.75);
  chassis.pid_wait_until(14.5_in);
  matchloaderActive(true);
  pros::delay(50);
  chassis.pid_wait_until(27.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();



  chassis.pid_turn_set(-103_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(-56.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true, true);

  
  chassis.pid_drive_set(28.5_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(12_in);
  reverseScoring(true);
  intake(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(790);

  chassis.pid_drive_set(-32.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(820);

  matchloaderInactive(true);
  scoreHigh(true); // scores in the high goal
  pros::delay(1500); 

  turnOffScoring(true);
  intake(false);

  setAntiJam(false, false);

  // descore starting on long goal

  wingToggle();

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(41_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  pros::delay(50);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait_quick_chain();

}

void Left4BallRush(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
  wingToggle();
  intake(true); // turns on intake

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true);  // moves towards triple balls
  chassis.pid_wait_until(8_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.75);
  chassis.pid_wait_until(14.5_in);
  matchloaderActive(true);
  pros::delay(50);
  chassis.pid_wait_until(27.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait();



  chassis.pid_turn_set(-103_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(-57_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);
  setAntiJam(true, true);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED*0.9, true);
  pros::delay(180);

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  matchloaderInactive(true);

  scoreHigh(true); // scores in the high goal
  pros::delay(990); 


  setAntiJam(false, false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // descore starting on long goal


  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  wingToggle();


  chassis.pid_turn_set(-43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-9_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(39_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  pros::delay(50);

  chassis.pid_drive_set(-22_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait();

  
}

void Left3Plus4(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);

  setMatch(false);
  setAntiJam(false, false);
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

void progSkills81(){
  setMatch(false); // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false, false);

  wingToggle();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32.5_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();

  chassis.pid_turn_set(-91.5_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  matchloaderActive(true);
  intake(true);
  
  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(12.75_in,DRIVE_SPEED*0.6,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(900); // first matchloader delay


  chassis.pid_drive_set(-32.75_in,DRIVE_SPEED*0.75,true); // align with goal-check
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  pros::delay(50);

  intake(false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);




  chassis.pid_turn_set(47_deg,TURN_SPEED*0.9,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(14.5_in,DRIVE_SPEED,true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(-49_deg,TURN_SPEED*0.9,false); // turn into alley, aligned to drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-75_in,DRIVE_SPEED*0.82,true); // drive across alley
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
  
  chassis.pid_drive_set(-15_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(127_deg, TURN_SPEED, false); // align to score on goal for first time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in,DRIVE_SPEED*0.75,true); // drive into goal
  pros::delay(705);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  // extake(true);
  // pros::delay(150);
  setAntiJam(true, true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);  // score for the first time
  pros::delay(1700);
  setAntiJam(false, false);

  

  pros::delay(15);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(34.75_in,DRIVE_SPEED*0.75,true); // drive into 2nd matchloader
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  chassis.pid_wait_until(15.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait();
  intake(true);
  

  pros::delay(1300); // second matchloader delay

  chassis.pid_drive_set(-32_in,DRIVE_SPEED*0.5,true); // drive into goal
  pros::delay(880);
  //chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true, true);
  extake(true);
  pros::delay(100);
  scoreHigh(true); // score into goal second time
  pros::delay(2300);
  turnOffScoring(true);
  setAntiJam(false, false);
  turnOffLower(true);

  matchloaderInactive(true); // up

  

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  
  intake(true);
  turnOffScoring(true);
  turnOffLower(true);

  pros::delay(100);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);







  // start middle goal 




  
  chassis.pid_turn_set(91_deg, TURN_SPEED*0.9, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(71_in, DRIVE_SPEED*0.6, true);
  chassis.pid_wait_until(3.5_in);
  //matchloaderActive(true);
  pros::delay(250);
  //matchloaderInactive(true);
  chassis.pid_wait_until(45_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait_until(50_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-8.5_in, DRIVE_SPEED*0.9, true);
  chassis.pid_wait_until(-4.5_in);
  matchloaderInactive(true);
  chassis.pid_wait();


  chassis.pid_turn_set(-47_deg, TURN_SPEED, false); // turn to face middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(-18_in, DRIVE_SPEED*0.75, true); // drive into middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(125);

  middleGoalPiston(true);
  MiddleGoalScoreSkills(true);

  pros::delay(1250);

  
  middleGoalPiston(false);
  scoreHigh(true);


 

  chassis.pid_drive_set(52.75_in, DRIVE_SPEED, true); // lurch into goal/matchload area
  pros::delay(1200);

  reverseScoring(true);
  intake(true);

 


  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to face long goal for align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(-24_in,DRIVE_SPEED*0.9,true); // align with goal for goal check
  pros::delay(800);
  

  //end middle goal


  matchloaderActive(true); // down

  pros::delay(50);

  intake(true); 

  chassis.pid_drive_set(30.75_in,DRIVE_SPEED*0.75,true); // third matchloader
  chassis.pid_wait_until(15.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait();

  
  pros::delay(800); // 3rd matchloader delay


  chassis.pid_drive_set(-31_in,DRIVE_SPEED*0.52,true); // align with goal for goal check
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true); // u

  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal check
  chassis.pid_wait();

  intake(false);

  chassis.pid_turn_set(43_deg,TURN_SPEED,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(16_in,DRIVE_SPEED,true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(-45.5_deg,TURN_SPEED,false); // turn to be parallel to alley and drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-76_in,DRIVE_SPEED*0.84,true); // drive across alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_set(13_deg, TURN_SPEED, false); // turn to move out of alley 
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-12.75_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to realign
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-14.5_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(125.5_deg, TURN_SPEED, false); // turn to align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-23_in,DRIVE_SPEED*0.6,true); // drive into goal for 3rd score
  pros::delay(850);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  matchloaderActive(true); // d

  setAntiJam(true, true);
  extake(true);
  pros::delay(100);
  scoreHigh(true); // score goal for the third time
  pros::delay(1850);
  setAntiJam(false, false);


  
  

  chassis.pid_drive_set(31_in,DRIVE_SPEED*0.75,true); // drive into 4th matchloader
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait_until(15.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait();

  
  pros::delay(800); // 4th matchloader delay

  
  chassis.pid_drive_set(-32.5_in,DRIVE_SPEED*0.5,true); // drive into goal and for final score
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true, true);

  //holy  sequence
  extake(true);
  pros::delay(25);
  extake(false);
  pros::delay(25);
  extake(true);
  pros::delay(25);
  extake(false);
  pros::delay(25);
  extake(true);
  pros::delay(25);
  extake(false);
  pros::delay(25);


  scoreHigh(true);
  pros::delay(1700);
  setAntiJam(false, false);
  turnOffScoring(true);
  
  matchloaderInactive(true); // u

  
  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);
  turnOffScoring(true);
  turnOffLower(true);
  pros::delay(100);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);


  




  //park

  intake(true);

  //turn on intake to prepare for park clear
  

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 85});
  chassis.pid_wait();

  chassis.pid_drive_set(3_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait_quick();

  chassis.pid_drive_set(28_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait_quick();

}

void progSkills75(){
  setMatch(false); // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false, false);

  wingToggle();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32.5_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  matchloaderActive(true);
  intake(true);
  
  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(12.75_in,DRIVE_SPEED*0.6,true); // move into first matchloader to empty
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  pros::delay(800); // first matchloader delay


  chassis.pid_drive_set(-32.75_in,DRIVE_SPEED*0.75,true); // align with goal-check
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  pros::delay(50);

  intake(false);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);




  chassis.pid_turn_set(47_deg,TURN_SPEED*0.9,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
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
  
  chassis.pid_drive_set(-15_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(125_deg, TURN_SPEED, false); // align to score on goal for first time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in,DRIVE_SPEED*0.75,true); // drive into goal
  pros::delay(705);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  // extake(true);
  // pros::delay(150);
  setAntiJam(true, true);
  extake(true);
  pros::delay(200);
  scoreHigh(true);  // score for the first time
  pros::delay(1700);
  setAntiJam(false, false);

  

  pros::delay(15);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(34.75_in,DRIVE_SPEED*0.35,true); // drive into 2nd matchloader
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait();

  pros::delay(1400); // second matchloader delay

  chassis.pid_drive_set(-32_in,DRIVE_SPEED*0.5,true); // drive into goal
  pros::delay(880);
  //chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true, true);
  extake(true);
  pros::delay(200);
  scoreHigh(true); // score into goal second time
  pros::delay(2200);
  turnOffScoring(true);
  setAntiJam(false, false);

  matchloaderInactive(true); // up

  

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(15_in,DRIVE_SPEED*0.7,true); // drive away from goal
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  chassis.pid_wait();

  intake(false);

  chassis.pid_turn_set(89_deg, TURN_SPEED, false); //turn to other side
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(96_in,DRIVE_SPEED*0.7,true); // cross field
  chassis.pid_wait();

  chassis.pid_turn_set(-89_deg, TURN_SPEED, false); //turn to face goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  chassis.pid_drive_set(-19.5_in,DRIVE_SPEED*0.75,true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true); // down

  pros::delay(50);

  intake(true); 

  chassis.pid_drive_set(30.5_in,DRIVE_SPEED*0.35,true); // third matchloader
  chassis.pid_wait();

  
  pros::delay(800); // 3rd matchloader delay


  chassis.pid_drive_set(-31_in,DRIVE_SPEED*0.52,true); // align with goal for goal check
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true); // u

  chassis.pid_drive_set(10_in,DRIVE_SPEED,true); // drive out of goal check
  chassis.pid_wait();

  intake(false);

  chassis.pid_turn_set(43_deg,TURN_SPEED,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
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

  chassis.pid_drive_set(-12.75_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to realign
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-14.5_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(125_deg, TURN_SPEED, false); // turn to align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-23_in,DRIVE_SPEED*0.5,true); // drive into goal for 3rd score
  pros::delay(900);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  matchloaderActive(true); // d

  setAntiJam(true, true);
  extake(true);
  pros::delay(200);
  scoreHigh(true); // score goal for the third time
  pros::delay(1850);
  setAntiJam(false, false);


  
  

  chassis.pid_drive_set(30.5_in,DRIVE_SPEED*0.40,true); // drive into 4th matchloader
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait();

  
  pros::delay(800); // 4th matchloader delay

  
  chassis.pid_drive_set(-35_in,DRIVE_SPEED*0.5,true); // drive into goal and for final score
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

    setAntiJam(true, true);
  extake(true);
  pros::delay(200);
  scoreHigh(true);
  pros::delay(1700);
  setAntiJam(false, false);
  turnOffScoring(true);
  
  matchloaderInactive(true); // u


  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // extake(true); // score into goal for fourth time
  // pros::delay(200);




  // park

  // turn on intake to prepare for park clear
  

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 73});
  chassis.pid_wait();

  chassis.pid_drive_set(32_in, DRIVE_SPEED*0.82, true);
  chassis.pid_wait();

}


void SOAP(){
  chassis.pid_turn_chain_constant_set(2_deg);
  chassis.pid_drive_chain_constant_set(2_in);
  setMatch(false);
  setAntiJam(false, false);

  intake(true); // turns on intake


  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  // // first long goal scoring

  chassis.pid_drive_set(32.5_in,DRIVE_SPEED*0.9,true); // drive off the park area
  chassis.pid_wait();
  
  intake(true);

  matchloaderActive(true);

  chassis.pid_turn_set(92.5_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(7.5_in,DRIVE_SPEED*0.65,true); // move into first matchloader to empty
  pros::delay(850); // manual timeout for speed

  chassis.pid_drive_set(-30.5_in,DRIVE_SPEED,true); // align with goal-check
  chassis.pid_wait_until(-22_in);
  scoreHigh(true);
  pros::delay(700);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);
  pros::delay(700);

  intake(false);

  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);

  intake(true);

  chassis.pid_drive_set(69_in, DRIVE_SPEED*0.75, true);
  chassis.pid_wait_until(27_in);
  matchloaderActive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-8.25_in, DRIVE_SPEED*0.9, true);
  chassis.pid_wait();



  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to face middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(-18.5_in, DRIVE_SPEED*0.75, true); // drive into middle goal
  pros::delay(570);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(50);
  setAntiJam(true, true);

  
  matchloaderActive(true);

  middleGoalPiston(true);
  MiddleGoalScoreSkills(true);

  setAntiJam(false, false);

  pros::delay(670);

  MiddleGoalScoreSkills(true);

  reverseScoring(true);
  intake(true);

  //end middle goal scoring

  chassis.pid_drive_set(57_in, DRIVE_SPEED, true); // lurch into goal/matchload area
  chassis.pid_wait_until(10_in);
  middleGoalPiston(false);
  pros::delay(1150);

  chassis.pid_turn_set(-44.5_deg, TURN_SPEED, false); // turn to face long goal for align
  pros::delay(485);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  // chassis.pid_drive_set(-8_in,DRIVE_SPEED*0.9,true); // align with goal for goal check
  // pros::delay(450);

  chassis.pid_drive_set(7_in,DRIVE_SPEED*0.9,true); // align with goal for goal check
  pros::delay(750);
  
  chassis.pid_drive_set(-36_in,DRIVE_SPEED*0.9,true); // align with goal for goal check
  chassis.pid_wait_until(-20_in);
  scoreHigh(true);
  pros::delay(570);
  














  //^^^^^ OLD SAWP CAN DELETE ONCE DONE
  // //1010G sawp try
  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // intake(true);

  // chassis.pid_drive_set(69_in, DRIVE_SPEED*0.65, true);
  // chassis.pid_wait_until(3.5_in);
  // //matchloaderActive(true);
  // pros::delay(250);
  // //matchloaderInactive(true);
  // chassis.pid_wait_until(50_in);
  // //matchloaderActive(true);
  // pros::delay(250);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(-5.25_in, DRIVE_SPEED*0.9, true);
  // chassis.pid_wait_until(-2_in);
  // //matchloaderInactive(true);
  // chassis.pid_wait();


  // chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to face middle goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // intake(false);
  

  // reverseScoring(true);
  // intake(true);

 

  // chassis.pid_drive_set(32.25_in, DRIVE_SPEED, true); // lurch into goal/matchload area
  // pros::delay(1200);

  // chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to face long goal for align
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  // chassis.pid_drive_set(-24_in,DRIVE_SPEED*0.9,true); // align with goal for goal check
  // pros::delay(750);
  

  // scoreHigh(true);
  // matchloaderActive(true);

  // pros::delay(700);

  // setAntiJam(false, false);

  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(32_in, DRIVE_SPEED, true); 
  // chassis.pid_wait_until(16_in);
  // reverseScoring(true);
  // intake(true);
  // chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  // pros::delay(800);

  // chassis.pid_drive_set(-11_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  // chassis.pid_wait();

  // matchloaderInactive(true);

  // chassis.pid_turn_set(47_deg, TURN_SPEED, false); // turn to matchloader
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  // chassis.pid_drive_set(-50.5_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  // chassis.pid_wait();

  // middleGoalPiston(true);
  //scoreMid(true, false);


}

void SingleSideSOAP(){

}

void progSkills96(){

  setMatch(false); // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false, false);

  wingToggle();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(10.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait();

  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED, true);  // align with goal
  chassis.pid_wait();

  chassis.pid_turn_set(-116_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  pros::delay(820);


  middleGoalPiston(true);
  MiddleGoalScoreSkills(true);
  matchloaderActive(true); // prepare for matchload


  pros::delay(400);

  middleGoalPiston(false);
  MiddleGoalScoreSkills(false);

  intake(true);

  chassis.pid_drive_set(53.5_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  

  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  chassis.pid_drive_set(14.5_in, DRIVE_SPEED*0.45, true); // drive towards matchloader
  chassis.pid_wait();
  pros::delay(850); // unloads matchloader







  // matchloader and onwards
  
  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); // drive forward to align with matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn towards matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17_in, DRIVE_SPEED, true); // drive into alley
  chassis.pid_wait();

  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to be parallel to alley and drive back
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(70_in, DRIVE_SPEED, true); // drive across alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_turn_set(90_deg, TURN_SPEED, false); // turn to move out of alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(15_in, DRIVE_SPEED, true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(-90_deg, TURN_SPEED, false); // turn to realign
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-15_in, DRIVE_SPEED, true); // drive out of alley
  chassis.pid_wait();

  pros::delay(1000000);

  chassis.pid_drive_set(-22.75_in,DRIVE_SPEED*0.75,true); // align with goal-check
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true);

  intake(false);


  chassis.pid_turn_set(47_deg,TURN_SPEED*0.9,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
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
  
  chassis.pid_drive_set(-15_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(127_deg, TURN_SPEED, false); // align to score on goal for first time
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-22_in,DRIVE_SPEED*0.75,true); // drive into goal
  pros::delay(705);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);
  
  // extake(true);
  // pros::delay(150);
  setAntiJam(true, true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);  // score for the first time
  pros::delay(1700);
  setAntiJam(false, false);

  

  pros::delay(15);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(34.75_in,DRIVE_SPEED*0.75,true); // drive into 2nd matchloader
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  chassis.pid_wait_until(15.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait();
  intake(true);
  

  pros::delay(1200); // second matchloader delay

  chassis.pid_drive_set(-32_in,DRIVE_SPEED*0.5,true); // drive into goal
  pros::delay(880);
  //chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true, true);
  extake(true);
  pros::delay(100);
  scoreHigh(true); // score into goal second time
  pros::delay(2300);
  turnOffScoring(true);
  setAntiJam(false, false);

  matchloaderInactive(true); // up

  

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);
  pros::delay(100);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);







  // start park clear 

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 73});
  chassis.pid_wait();

  chassis.pid_drive_set(3_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait();


  distanceCorrection(34, 558, DRIVE_SPEED, FRONT);

  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();

  chassis.pid_drive_set(66_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait();

  
  chassis.pid_turn_set(-135_deg, TURN_SPEED, false);
  chassis.pid_wait();

  chassis.pid_drive_set(-20_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait();


  
  extake(true);
  pros::delay(125);
  setAntiJam(true, true);

  middleGoalPiston(true);
  MiddleGoalScoreSkills(true);

  setAntiJam(false, false);

  pros::delay(1750);

  middleGoalPiston(false);
  MiddleGoalScoreSkills(false);

  reverseScoring(true);
  intake(true);

 

  chassis.pid_drive_set(52.75_in, DRIVE_SPEED, true); // lurch into goal/matchload area
  pros::delay(1200);

    matchloaderActive(true); // down


  chassis.pid_turn_set(-46_deg, TURN_SPEED, false); // turn to face long goal for align
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  

  //end middle goal




  intake(true); 

  chassis.pid_drive_set(6.75_in,DRIVE_SPEED*0.35,true); // third matchloader
  chassis.pid_wait();

  
  pros::delay(800); // 3rd matchloader delay


  chassis.pid_drive_set(-21_in,DRIVE_SPEED*0.52,true); // align with goal for goal check
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderInactive(true); // u


  intake(false);

  chassis.pid_turn_set(43_deg,TURN_SPEED,false); // turn towards the alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
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

  chassis.pid_drive_set(-12.75_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(45_deg, TURN_SPEED, false); // turn to realign
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-15.5_in,DRIVE_SPEED,true); // drive out of alley
  chassis.pid_wait();
  
  chassis.pid_turn_set(125_deg, TURN_SPEED, false); // turn to align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-23_in,DRIVE_SPEED*0.5,true); // drive into goal for 3rd score
  pros::delay(900);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  matchloaderActive(true); // d

  setAntiJam(true, true);
  extake(true);
  pros::delay(100);
  scoreHigh(true); // score goal for the third time
  pros::delay(1850);
  setAntiJam(false, false);


  
  

  chassis.pid_drive_set(30.5_in,DRIVE_SPEED*0.75,true); // drive into 4th matchloader
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  intake(true);
  chassis.pid_wait_until(15.5_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  chassis.pid_wait();

  
  pros::delay(800); // 4th matchloader delay

  
  chassis.pid_drive_set(-35_in,DRIVE_SPEED*0.5,true); // drive into goal and for final score
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  setAntiJam(true, true);

  //holy  sequence
  extake(true);
  pros::delay(100);


  scoreHigh(true);
  pros::delay(1700);
  setAntiJam(false, false);
  turnOffScoring(true);
  
  matchloaderInactive(true); // u


  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);
  turnOffScoring(true);
  pros::delay(50);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);






  //park

  //turn on intake to prepare for park clear
  

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  turnOffScoring(true);
  extake(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 73});
  chassis.pid_wait();

  chassis.pid_drive_set(3_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait();

  chassis.pid_drive_set(28_in, DRIVE_SPEED*0.80, true);
  chassis.pid_wait();

}

void progSkills79(){
  setMatch(false); // initializes to turn off color sort and anti-jam to reduce inconsistency
  setAntiJam(false, false);

  wingToggle();

   chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.odom_boomerang_dlead_set(0.3);

  // get quartet

  chassis.pid_odom_set({{{-0_in, 15_in}, fwd, DRIVE_SPEED},
                        {{-5_in, 20_in}, fwd, 120},
                        {{-10_in, 32_in}, fwd, 110}},
                       true);

  chassis.pid_wait_until(18_in);
  matchloaderActive(true);
  chassis.pid_wait();
  
  chassis.odom_xyt_set(0_in, 0_in, -23.35_deg);

  chassis.pid_turn_set(-135_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-19.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  pros::delay(500);


  middleGoalPiston(true);

  MiddleGoalScoreSkills(true);
  pros::delay(300); // delay for mid score
  fastSkillsMidScore(true);
  pros::delay(1000);

  middleGoalPiston(false);
  fastSkillsMidScore(false);


  chassis.pid_drive_set(54.75_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  pros::delay(20);

  if(frontD.get() > 535 || frontD.get() < 525){
    float DistanceToMove = frontD.get() - 530;
    DistanceToMove *= 0.03937008;
    chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
    chassis.pid_wait();
  }

  intake(true);

  chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(9.25_in, DRIVE_SPEED*0.7, true); // drive towards matchloader
  chassis.pid_wait();
  pros::delay(850); // unloads matchloader
  
  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true); // drive away from matchloader
  chassis.pid_wait();
  matchloaderInactive(true);
  
  chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);

  chassis.pid_drive_set(13.5_in, DRIVE_SPEED, true); // drive towards alley
  chassis.pid_wait();

  chassis.drive_set(DRIVE_SPEED*0.5, DRIVE_SPEED*0.3); // curve into the alley
  pros::delay(750);
  chassis.drive_set(0, 0);



  chassis.drive_set(DRIVE_SPEED*0.95, DRIVE_SPEED); // drive across alley
  pros::delay(800);
  chassis.drive_set(DRIVE_SPEED*0.65, DRIVE_SPEED*0.7); // drive across alley
  pros::delay(125);
  chassis.drive_set(DRIVE_SPEED*0.35, DRIVE_SPEED*0.4); // drive across alley
  pros::delay(75);
  chassis.drive_set(0, 0);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  // add distance sensor check here

  pros::delay(20);

  if(frontD.get() > 655 || frontD.get() < 645){
    float DistanceToMove = frontD.get() - 650;
    DistanceToMove *= 0.03937008;
    chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
    chassis.pid_wait();
  }


  
  //turn is 135 bc that makes it 90 when hitting the wall
  chassis.pid_turn_set(90_deg, TURN_SPEED, false); // turn out of alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-7.25_in, DRIVE_SPEED*0.5, true); // wall check
  pros::delay(350);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17_in, DRIVE_SPEED, true); // drive infront of goal
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED, false); // turn to goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-18.0_in, DRIVE_SPEED*0.90, true); // drive to goal
  pros::delay(500);

  matchloaderActive(true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1475);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  wingToggle();

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 2nd matchloader
  chassis.pid_wait_until(15_in);
  intake(true);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED*0.75, true); // drive into goal for second score
  pros::delay(500);

  matchloaderInactive(true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1800);
  scoreHigh(false);

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);

  chassis.pid_drive_set(-5.5_in,DRIVE_SPEED*0.75, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(15_in,DRIVE_SPEED*0.7,true); // drive away from goal
  chassis.pid_wait();

  intake(false);

  chassis.pid_turn_set(89_deg, TURN_SPEED, false); //turn to other side
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(97_in,DRIVE_SPEED*0.7,true); // cross field
  chassis.pid_wait();

  chassis.pid_turn_set(-89_deg, TURN_SPEED, false); //turn to face goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);

  chassis.pid_drive_set(-21.5_in,DRIVE_SPEED*0.75,true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 3rd matchloader
  chassis.pid_wait();

  pros::delay(1000);


  chassis.pid_drive_set(-10_in, DRIVE_SPEED, true); // drive away from matchloader
  chassis.pid_wait();
  matchloaderInactive(true);
  
  chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);

  chassis.pid_drive_set(13.5_in, DRIVE_SPEED, true); // drive towards alley
  chassis.pid_wait();

  chassis.drive_set(DRIVE_SPEED*0.5, DRIVE_SPEED*0.3); // curve into the alley
  pros::delay(750);
  chassis.drive_set(0, 0);



  chassis.drive_set(DRIVE_SPEED*0.95, DRIVE_SPEED); // drive across alley
  pros::delay(800);
  chassis.drive_set(DRIVE_SPEED*0.65, DRIVE_SPEED*0.7); // drive across alley
  pros::delay(125);
  chassis.drive_set(DRIVE_SPEED*0.35, DRIVE_SPEED*0.4); // drive across alley
  pros::delay(75);
  chassis.drive_set(0, 0);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  // add distance sensor check here

  pros::delay(20);

  if(frontD.get() > 655 || frontD.get() < 645){
    float DistanceToMove = frontD.get() - 650;
    DistanceToMove *= 0.03937008;
    chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
    chassis.pid_wait();
  }


  
  //turn is 135 bc that makes it 90 when hitting the wall
  chassis.pid_turn_set(90_deg, TURN_SPEED, false); // turn out of alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-7.25_in, DRIVE_SPEED*0.5, true); // wall check
  pros::delay(350);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17_in, DRIVE_SPEED, true); // drive infront of goal
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED, false); // turn to goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-18.0_in, DRIVE_SPEED*0.90, true); // drive to goal
  pros::delay(500);

  matchloaderActive(true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1475);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  wingToggle();

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 2nd matchloader
  chassis.pid_wait_until(15_in);
  intake(true);
  chassis.pid_wait();


  pros::delay(1100);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED*0.75, true); // drive into goal for second score
  pros::delay(500);

  matchloaderInactive(true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1800);
  scoreHigh(false);

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);

  chassis.pid_drive_set(-5_in,DRIVE_SPEED*0.5, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  // park


  scoreHigh(true);

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 73});
  chassis.pid_wait();

  extake(true);

  chassis.pid_drive_set(32_in, DRIVE_SPEED*0.82, true);
  chassis.pid_wait();

}

