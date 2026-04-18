#include "autons.hpp"
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
#include <cstdint>
#include <iterator>

extern pros::Controller master;

extern pros::Distance frontD;
extern pros::Distance frontD2;


extern pros::adi::Pneumatics midDescore;
extern pros::adi::Pneumatics hood;
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
  chassis.pid_drive_constants_set(15, 0.0, 150);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(16.5, 0.5, 29);        // Holds the robot straight while going forward without odom
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

  // chassis.pid_turn_set(-45_deg, TURN_SPEED, false);
  // chassis.pid_wait();

  // pros::delay(50);

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);

  // chassis.pid_turn_set(180_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);

  // chassis.pid_turn_set(180_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);

  // chassis.pid_turn_set(-90_deg, TURN_SPEED, false);
  // chassis.pid_wait();
  // //chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(50);





  
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



//AUTONS FOR PUSHBACK




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


void progSkills96(){

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
  pros::delay(100);

  middleGoalPiston(false);
  fastSkillsMidScore(false);


  chassis.pid_drive_set(54.25_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  pros::delay(20);

  if(frontD.get() > 540 || frontD.get() < 520){
    float DistanceToMove = frontD.get() - 542.7;
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

  chassis.pid_drive_set(13.25_in, DRIVE_SPEED, true); // drive infront of goal
  pros::delay(650);

  chassis.pid_turn_set(-90_deg, TURN_SPEED, false); // turn to goal
  pros::delay(770);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-18.0_in, DRIVE_SPEED*0.90, true); // drive to goal
  pros::delay(500);

  matchloaderActive(true);
  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1775);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 2nd matchloader
  chassis.pid_wait_until(15_in);
  wingToggle();
  intake(true);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(-31_in, DRIVE_SPEED*0.75, true); // drive into goal for second score
  pros::delay(500);


  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(2075);
  scoreHigh(false);
  matchloaderInactive(true);

  pros::delay(100);

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);

  chassis.pid_drive_set(-5.5_in,DRIVE_SPEED*0.75, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);





  
  
  // clear park + get 6 red balls






  scoreHigh(true);

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  

  chassis.pid_odom_set({{26.5_in, 33_in, 90_deg}, fwd, 73});
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(67_in, DRIVE_SPEED*0.62, true);
  chassis.pid_wait_until(42_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.35);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-8_in, DRIVE_SPEED*0.25, true);
  chassis.pid_wait();
   chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);

  // pros::delay(2500);

  // if(frontD2.get() > 360 || frontD2.get() < 200){
  //   float DistanceToMove = frontD2.get() - 250;
  //   DistanceToMove *= 0.03937008;
  //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
  //   chassis.pid_wait();
  // }


  
  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-5.5_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(36_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-135_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-25.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  chassis.pid_wait();

  extake(true);
  pros::delay(100);

  matchloaderInactive(true);
  middleGoalPiston(true);
  wingToggle();
  
  turnLoweratXSpeed(true, 127*1);
  turnScoringatXSpeed(true, 127*0.5);


  pros::delay(1400);

 chassis.pid_drive_set(0.5_in, DRIVE_SPEED, true);  // moves towards middle goal
 chassis.pid_wait();

  pros::delay(500);


  middleGoalPiston(false);
  MiddleGoalScoreSkills(false);

  reverseScoring(true);
  intake(true);
  matchloaderActive(true);

  // end the peak middle goal section


  chassis.pid_drive_set(53_in, DRIVE_SPEED*0.95, true); // lurch into goal/matchload area
  chassis.pid_wait();

//   pros::delay(1000);

//   if(frontD.get() > 680 || frontD.get() < 670){
//     float DistanceToMove = frontD.get() - 675;
//     DistanceToMove *= 0.03937008;
//     chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.75, true);
//     chassis.pid_wait();
//   }

  intake(true);

  chassis.pid_turn_set(-46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(9.50_in, DRIVE_SPEED*0.7, true); // drive towards 3rd matchloader
  chassis.pid_wait();
  pros::delay(850); // unloads matchloader
  
  chassis.pid_drive_set(-9.5_in, DRIVE_SPEED, true); // drive away from matchloader
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
    chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.75, true);
    chassis.pid_wait();
  }


  
  //turn is 135 bc that makes it 90 when hitting the wall
  chassis.pid_turn_set(90_deg, TURN_SPEED, false); // turn out of alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(13.25_in, DRIVE_SPEED, true); // drive infront of goal
  pros::delay(650);

  chassis.pid_turn_set(-90_deg, TURN_SPEED, false); // turn to goal
  pros::delay(770);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);

  chassis.pid_drive_set(-18.0_in, DRIVE_SPEED*0.90, true); // drive to goal
  pros::delay(500);

  extake(true);
  pros::delay(150);
  scoreHigh(true);
  pros::delay(1675);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.25_in, DRIVE_SPEED*0.7, true); // drive into 4th matchloader
  chassis.pid_wait_until(15_in);
  wingToggle();
  intake(true);
  chassis.pid_wait();

  pros::delay(1000);

  chassis.pid_drive_set(-30.5_in, DRIVE_SPEED*0.75, true); // drive into goal for 4th score
  pros::delay(550);


  extake(true);
  pros::delay(150);
  scoreHigh(true);
  pros::delay(2075);
  scoreHigh(false);
  matchloaderInactive(true);

  pros::delay(100);

  chassis.pid_drive_set(5_in,DRIVE_SPEED, true); // align with goal
  chassis.pid_wait();

  intake(true);

  chassis.pid_drive_set(-5.5_in,DRIVE_SPEED*0.75, true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  // park


  scoreHigh(true);

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  

  chassis.pid_odom_set({{26.5_in, 33.5_in, 90_deg}, fwd, DRIVE_SPEED});
  chassis.pid_wait();

  extake(true);

  chassis.pid_drive_set(32_in, DRIVE_SPEED*0.82, true);
  chassis.pid_wait();

}


void testHeadingCorrect(){
  chassis.drive_imu_reset();  
  double heading = chassis.drive_imu_get();
  master.print(0, 0, "Heading: %f", heading);



  chassis.pid_turn_set(91_deg, TURN_SPEED, false);
  chassis.pid_wait();

  heading = chassis.drive_imu_get();



  chassis.odom_xyt_set(0_in, 0_in, heading * 1_deg);

  pros::delay(1000);

  master.print(1, 0, "Heading: %f", heading);


  chassis.pid_turn_set(90_deg, TURN_SPEED, false);
 chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(69_in, DRIVE_SPEED*0.50, true);
  chassis.pid_wait_until(50_in);
  matchloaderActive(true);
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
  pros::delay(1500);

  middleGoalPiston(false);
  fastSkillsMidScore(false);


  chassis.pid_drive_set(54.25_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();

  pros::delay(20);

  // if(frontD.get() > 540 || frontD.get() < 520){
  //   float DistanceToMove = frontD.get() - 542.7;
  //   DistanceToMove *= 0.03937008;
  //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
  //   chassis.pid_wait();
  // }

  intake(true);

  chassis.pid_turn_set(-45_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(9.25_in, DRIVE_SPEED*0.7, true); // drive towards matchloader
  chassis.pid_wait();
  pros::delay(1100);
  
  chassis.pid_drive_set(-9.75_in, DRIVE_SPEED, true); // drive away from matchloader
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
  pros::delay(1775);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 2nd matchloader
  chassis.pid_wait_until(15_in);
  wingToggle();
  intake(true);
  chassis.pid_wait();

  pros::delay(1100);

  chassis.pid_drive_set(-30.75_in, DRIVE_SPEED*0.75, true); // drive into goal for second score
  pros::delay(500);


  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(2075);
  scoreHigh(false);
  matchloaderInactive(true);

  pros::delay(100);

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

  chassis.pid_drive_set(-20.5_in,DRIVE_SPEED*0.75,true); // align with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 3rd matchloader
  chassis.pid_wait();

  pros::delay(1200);


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
  pros::delay(1875);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  chassis.pid_drive_set(30.5_in, DRIVE_SPEED*0.7, true); // drive into 4th matchloader
  chassis.pid_wait_until(15_in);
  wingToggle();
  intake(true);
  chassis.pid_wait();


  pros::delay(1200);

  chassis.pid_drive_set(-30.75_in, DRIVE_SPEED*0.75, true); // drive into goal for 4th score
  pros::delay(500);


  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(2175);
  scoreHigh(false);
  matchloaderInactive(true);

  pros::delay(100);


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















// worlds autos


void SoapLong(){
  setMatch(false);
  setAntiJamActiveWorlds(false);

  turnScoringatXSpeed(true, 127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 127);

  

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  // // first long goal scoring

  chassis.pid_drive_set(-39_in,DRIVE_SPEED,true); // drive off the park area
  chassis.pid_wait();

  matchloaderActive(true);

  chassis.pid_turn_set(-89_deg,TURN_SPEED,false); // turn towards the matchloader
  pros::delay(700);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(11.25_in,DRIVE_SPEED,true); // move into first matchloader to empty
  pros::delay(820);



  chassis.pid_drive_set(-31.5_in,DRIVE_SPEED,true); // align with goal-check
  pros::delay(750);
  hood.extend();
  turnScoringatXSpeed(true, 127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 127);
  matchloaderInactive(true);
  pros::delay(650); // first scoring delay
  turnScoringatXSpeed(true, 0);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);



  chassis.pid_turn_set(91_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in,0_deg);


  hood.retract();

  
  turnScoringatXSpeed(true, 127);


  chassis.pid_drive_set(63_in, DRIVE_SPEED*0.5, true); // drive across and get 6 blocks
  chassis.pid_wait_until(10_in);
  chassis.pid_speed_max_set(DRIVE_SPEED);
  chassis.pid_wait_until(58_in);
  matchloaderActive(true);
  chassis.pid_wait_until(62_in);
  matchloaderInactive(true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  


  chassis.odom_boomerang_dlead_set(0.3);


  chassis.pid_odom_set({{-20_in, 17.5_in, 267_deg}, fwd, 127}); // curve into the goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(-17.5_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(-14_in);
  hood.extend();
  pros::delay(200);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  matchloaderActive(true);

  
  hood.extend();
  turnScoringatXSpeed(true, 127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 127);

  
  pros::delay(600);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

 
  
  turnScoringatXSpeed(true, 0);
  turnLoweratXSpeed(true, 0);
  turnMiddleatXSpeed(true, 0);

  chassis.pid_drive_set(32.5_in, DRIVE_SPEED, true); // move into matchloader
  chassis.pid_wait_until(16_in);
  hood.retract();
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  turnScoringatXSpeed(true, 127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 10);
  hood.retract();
  pros::delay(825);

  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true); // moves out of matchloader
  pros::delay(460);

  matchloaderInactive(true);

  chassis.pid_turn_set(44_deg, TURN_SPEED, false); // turn to mid goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-55_in, DRIVE_SPEED, true);  // drive to mid goal
  chassis.pid_wait_until(-40_in);
  turnScoringatXSpeed(true, -127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 75);
  chassis.pid_wait_until(-53);
  midDescore.retract();
  chassis.pid_wait();
  

  


}

void TwoInch(){
  chassis.pid_drive_set(2_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

void Left4CurveRush(){
  wingToggle();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  turnScoringatXSpeed(true, 127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 127);

  

  chassis.odom_boomerang_dlead_set(0.3);

  // im using 127 bc it doesnt acc run at 127 bc it slows down to curve
  chassis.pid_odom_set({{{0_in, 15_in}, fwd, 127},
                      {{-5_in, 20_in}, fwd, 127},
                      {{-10_in, 26_in}, fwd, 127},
                      {{-35_in, -2_in}, fwd, 127}},
                     true);
  pros::delay(2430);

  

  chassis.pid_odom_set({{{-32.5_in, 15_in}, rev, 127}}, true);
  pros::delay(720);

  chassis.pid_drive_set(-2_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  hood.extend();

  pros::delay(650);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  

  // chassis.pid_odom_set({{{-32.5_in, 15_in}, rev, 127}}, true);
  // pros::delay(720);

  // chassis.pid_drive_set(-2_in, DRIVE_SPEED, true);
  // chassis.pid_wait_quick_chain();

  // hood.extend();

  // pros::delay(650);

  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.drive_set(DRIVE_SPEED, -30); // curve into the alley
  // pros::delay(1000);
  // chassis.drive_set(0, 0);

  // wingToggle();

  // chassis.pid_drive_set(20_in, DRIVE_SPEED, true);
  // chassis.pid_wait_quick_chain();



  
  ///DESCORE PART FO;R LATER;

  chassis.pid_turn_set(-50_deg,TURN_SPEED,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(11_in,DRIVE_SPEED,true); // move into first matchloader to empty
  chassis.pid_wait();

  chassis.pid_turn_set(50_deg,TURN_SPEED,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-10_in,DRIVE_SPEED*0.1,true); // move into first matchloader to empty
  chassis.pid_wait();




  


  
  


}

void Right4CurveRush(){

}

void Left4BallMidRush(){

  int totalTime = 15000;
  uint32_t startTime = pros::millis();

  intake(true); // turns on intake
  reverseScoring(true);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED, true);  // drive back for right distance
  chassis.pid_wait();

  chassis.pid_turn_set(-116_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  pros::delay(820);

  // score on middle goal 
  
  turnLoweratXSpeed(true, 113);

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(800);

  middleGoalPiston(false);
  scoreMid(false, false);

  //end middle goal scoring




  while (pros::millis() - startTime < totalTime - 250){ // wait until 0.25 second left so we can camp mid goal and confirm mid goal control
    pros::delay(10);
  }


  // drive away to make sure that last ball count as scored
  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();


}

void Right4BallMidRush(){
  
  int totalTime = 15000;
  uint32_t startTime = pros::millis();

  intake(true); // turns on intake
  reverseScoring(true);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often
  
  chassis.pid_drive_set(34.5_in, DRIVE_SPEED, true);  // moves towards triple balls
  chassis.pid_wait_until(17_in);
  matchloaderActive(true);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_drive_set(-7.5_in, DRIVE_SPEED, true);  // move back to correct distance
  chassis.pid_wait();

  chassis.pid_turn_set(-116_deg, TURN_SPEED, false); // turn to middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED, true);  // moves towards middle goal
  pros::delay(820);

  //start middle goal scoring

  turnLoweratXSpeed(true, 113);

  middleGoalPiston(true);
  scoreMid(true, false);

  pros::delay(800);

  middleGoalPiston(false);
  scoreMid(false, false);

  //end middle goal scoring


  while (pros::millis() - startTime < totalTime - 250){ // wait until 0.25 second left so we can camp mid goal and confirm mid goal control
    pros::delay(10);
  }

  // move back to make final ball scoring


  chassis.pid_drive_set(10_in, DRIVE_SPEED, true);
  chassis.pid_wait();

}

void progSkills83States(){
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

  chassis.pid_turn_set(-135_deg, TURN_SPEED, false); // turn to matchloader area
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(35.25_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait();


  intake(true);

  chassis.pid_turn_set(-46_deg, TURN_SPEED, false); // turn to goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-23_in, DRIVE_SPEED*0.9, true); // move back into the goal
  chassis.pid_wait_until(-10_in);
  scoreHigh(true);
  chassis.pid_wait();

  
  pros::delay(400);
  scoreHigh(false); // score 5 balls into the long goal

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(30.25_in, DRIVE_SPEED*0.7, true); // empty first matchloader 
  chassis.pid_wait_until(15_in);
  intake(true);
  chassis.pid_wait();
  pros::delay(1100);

  
  chassis.pid_drive_set(-9.3_in, DRIVE_SPEED, true); // move back from matchloader
  chassis.pid_wait();
  matchloaderInactive(true);
  
  chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to alley
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

  // if(frontD.get() > 655 || frontD.get() < 645){
  //   float DistanceToMove = frontD.get() - 650;
  //   DistanceToMove *= 0.03937008;
  //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.75, true);
  //   chassis.pid_wait();
  // }


  
  //turn is 135 bc that makes it 90 when hitting the wall
  chassis.pid_turn_set(90_deg, TURN_SPEED, false); // turn out of alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-7.25_in, DRIVE_SPEED*0.5, true); // wall check
  pros::delay(370);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17.5_in, DRIVE_SPEED, true); // drive infront of goal
  chassis.pid_wait();

  chassis.pid_turn_set(-89_deg, TURN_SPEED, false); // turn to goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);

  chassis.pid_drive_set(-18.0_in, DRIVE_SPEED*0.90, true); // drive to goal
  pros::delay(500);

  //score first matchloader 
  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1675);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.7, true); // drive into 2nd matchloader
  chassis.pid_wait_until(15_in);
  wingToggle();
  intake(true);
  chassis.pid_wait();
  pros::delay(1200);

  chassis.pid_drive_set(-31.75_in, DRIVE_SPEED*0.75, true); // drive into goal for second score
  pros::delay(520);


  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(2075);
  scoreHigh(false);
  matchloaderInactive(true);

  pros::delay(100);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);




  //mid goal the final op


  chassis.pid_turn_set(91_deg, TURN_SPEED, false);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(69_in, DRIVE_SPEED*0.50, true);
  chassis.pid_wait_until(50_in);
  matchloaderActive(true);
  chassis.pid_wait();

  pros::delay(100);

  chassis.pid_drive_set(-4.25_in, DRIVE_SPEED*0.9, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_turn_set(-47_deg, TURN_SPEED, false); // turn to face middle goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(false);
  
  chassis.pid_drive_set(-22.75_in, DRIVE_SPEED*0.80, true); // drive into middle goal
  pros::delay(690);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  extake(true);
  pros::delay(100);

  
  middleGoalPiston(true);
  wingToggle();
  
  turnLoweratXSpeed(true, 127*0.9);
  turnScoringatXSpeed(true, 127*0.6);


  pros::delay(2100);

  middleGoalPiston(false);
  MiddleGoalScoreSkills(false);

  reverseScoring(true);
  intake(true);

  //end middle goal scoring

  chassis.pid_drive_set(53.4_in, DRIVE_SPEED*0.95, true); // lurch into goal/matchload area
  chassis.pid_wait();

  // if(frontD.get() > 705 || frontD.get() < 688){
  //   float DistanceToMove = frontD.get() - 675;
  //   DistanceToMove *= 0.03937008;
  //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.75, true);
  //   chassis.pid_wait();
  // }

  intake(true);

  chassis.pid_turn_set(-48_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(10.15_in, DRIVE_SPEED*0.7, true); // drive towards 3rd matchloader
  chassis.pid_wait();
  pros::delay(1400);
  
  chassis.pid_drive_set(-10.15_in, DRIVE_SPEED, true); // drive away from matchloader
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

  // if(frontD.get() > 655 || frontD.get() < 645){
  //   float DistanceToMove = frontD.get() - 650;
  //   DistanceToMove *= 0.03937008;
  //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.75, true);
  //   chassis.pid_wait();
  // }


  
  //turn is 135 bc that makes it 90 when hitting the wall
  chassis.pid_turn_set(90_deg, TURN_SPEED, false); // turn out of alley
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(-7.25_in, DRIVE_SPEED*0.5, true); // wall check
  pros::delay(370);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(17_in, DRIVE_SPEED, true); // drive infront of goal
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED, false); // turn to goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  matchloaderActive(true);

  chassis.pid_drive_set(-18.0_in, DRIVE_SPEED*0.90, true); // drive to goal
  pros::delay(500);

  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(1675);
  scoreHigh(false);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(30.25_in, DRIVE_SPEED*0.7, true); // drive into 4th matchloader
  chassis.pid_wait_until(15_in);
  wingToggle();
  intake(true);
  chassis.pid_wait();
  pros::delay(1200);

  chassis.pid_drive_set(-30.5_in, DRIVE_SPEED*0.75, true); // drive into goal for 4th score
  pros::delay(550);


  extake(true);
  pros::delay(100);
  scoreHigh(true);
  pros::delay(2175);
  scoreHigh(false);
  matchloaderInactive(true);

  pros::delay(100);

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  // park


  scoreHigh(true);

  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait_until(10_in);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  chassis.odom_boomerang_dlead_set(0.3);

  

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 90});
  chassis.pid_wait();

  extake(true);

  chassis.pid_drive_set(33_in, DRIVE_SPEED*0.82, true); // park
  chassis.pid_wait();

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
  pros::delay(650);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(29.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  pros::delay(855);

  chassis.pid_turn_set(-57.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);

  chassis.pid_drive_set(-20.5_in, DRIVE_SPEED*0.9, true);
  pros::delay(200);


  scoreHigh(true); // scores in the high goal
  pros::delay(1040); 

  

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true); 
  chassis.pid_wait_until(16_in);
  reverseScoring(true);
  intake(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(775);

  chassis.pid_drive_set(-9.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(46_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(-53_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_until(-51_in);

  middleGoalPiston(true);
  pros::delay(50);
  scoreMid(true, false);

  pros::delay(650);

  middleGoalPiston(false);
  scoreMid(false, false);

  
  chassis.pid_drive_set(7_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_quick_chain();

  midDescoreToggle();

  chassis.pid_drive_set(-8_in, DRIVE_SPEED, true);  // lurch to matchload area
  chassis.pid_wait();


  chassis.pid_drive_set(16_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.drive_set(DRIVE_SPEED*0.85, DRIVE_SPEED*0.0);
  pros::delay(650);
  chassis.drive_set(0, 0);
  pros::delay(400);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  wingToggle();
  chassis.pid_drive_set(17_in, DRIVE_SPEED, true);
  chassis.pid_wait();




}

void LowInverse3Plus4(){
  setMatch(false);
  setAntiJam(false, false);

  intake(true); // turns on intake


  chassis.odom_xyt_set(0_in, 0_in, 0_deg); // resets odom position every so often


  // // first long goal scoring

  chassis.pid_drive_set(38_in,DRIVE_SPEED*0.9,true); // drive off the park area
  pros::delay(950);
  
  intake(true);

  matchloaderActive(true);

  chassis.pid_turn_set(90_deg,TURN_SPEED*0.9,false); // turn towards the matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(11.25_in,DRIVE_SPEED*0.70,true); // move into first matchloader to empty
  pros::delay(900); // manual timeout for speed

  chassis.pid_drive_set(-30.5_in,DRIVE_SPEED,true); // align with goal-check
  chassis.pid_wait_until(-25_in);
  scoreHigh(true);
  pros::delay(700);
  matchloaderInactive(true);
  pros::delay(400);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  intake(true);

  chassis.pid_drive_set(22_in, DRIVE_SPEED, true);
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(131.5_deg, TURN_SPEED, false); // turn to face 1st trio ball
  chassis.pid_wait_quick();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);


  chassis.pid_drive_set(46_in, DRIVE_SPEED, true); // drive into trio balls and low goal
  chassis.pid_wait_until(10);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.45); 
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  scoreLow(true);


















































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
  pros::delay(620);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(30.5_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(58.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);

  chassis.pid_drive_set(-19.5_in, DRIVE_SPEED*0.85, true);
  chassis.pid_wait_until(-1_in);
  scoreHigh(true);
  pros::delay(1000);
  wingToggle();

  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_drive_chain_constant_set(3_in);


  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(36.25_in, DRIVE_SPEED, true);  // matchloader part
  chassis.pid_wait_until(16_in);
  intake(true);
  turnOffScoring(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.3);
  pros::delay(870);

  chassis.pid_drive_set(-9.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  chassis.pid_wait_until(-7_in);
  intake(false);
  chassis.pid_wait();

  matchloaderInactive(true);

  chassis.pid_turn_set(133.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_drive_set(48.5_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  chassis.pid_wait_until(28.5_in);
  scoreLow(true);
  chassis.pid_wait();

  pros::delay(400);

  intake(true);

  turnOffScoring(true);

  chassis.pid_drive_set(-33.75_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-137_deg, TURN_SPEED, false);
  chassis.pid_wait();

  wingToggle();

  chassis.pid_drive_set(-27_in, DRIVE_SPEED*0.9, true);
  chassis.pid_wait();

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
  pros::delay(500);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32.75_in, DRIVE_SPEED, true);  // lurch to matchloader
  chassis.pid_wait();

  chassis.pid_turn_set(53.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  reverseScoring(false);

  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true); 
  chassis.pid_wait_until(10_in);
  reverseScoring(true);
  intake(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.2);
  pros::delay(600);

  chassis.pid_drive_set(-29.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(680);

  // extake(true);
  // pros::delay(50);
  scoreHigh(true);
  matchloaderInactive(true);
  pros::delay(1300); 

  turnOffScoring(true);
  intake(false);

  // descore starting on long goal

  wingToggle();
  turnLoweratXSpeed(true, 50);
  turnScoringatXSpeed(true, -50);
  
  // descore starting on long goal

  wingToggle();

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(-43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-12.75_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(41_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

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



  chassis.pid_turn_set(-106_deg, TURN_SPEED, false); // turn to matchloader area
  pros::delay(630);
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(32_in, DRIVE_SPEED, true);  // lurch to matchloader
  pros::delay(840);

  pros::delay(50);

  chassis.pid_turn_set(-54.5_deg, TURN_SPEED, false); // turn to matchloader
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  reverseScoring(false);

  
  chassis.pid_drive_set(31_in, DRIVE_SPEED*0.95, true); 
  chassis.pid_wait_until(10_in);
  reverseScoring(true);
  intake(true);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.2);
  pros::delay(600);

  chassis.pid_drive_set(-29.5_in, DRIVE_SPEED, true); //moves toward the goal to do final score
  pros::delay(660);

  // extake(true);
  // pros::delay(50);
  scoreHigh(true);
  matchloaderInactive(true);
  pros::delay(1300); 

  turnOffScoring(true);
  intake(false);

  // descore starting on long goal

  wingToggle();
  turnLoweratXSpeed(true, 50);
  turnScoringatXSpeed(true, -50);
  
  // descore starting on long goal

  wingToggle();

  chassis.pid_drive_set(12_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(43_deg, TURN_SPEED, false); // turn out of goal
  chassis.pid_wait_quick_chain();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  chassis.pid_drive_set(-19_in, DRIVE_SPEED, true); // drive out of goal
  chassis.pid_wait_quick_chain();

  chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to be parallel with goal
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  chassis.pid_drive_set(19_in, DRIVE_SPEED, true); // descore balls in goal
  chassis.pid_wait_until(2_in);
  chassis.pid_speed_max_set(DRIVE_SPEED*0.5);
  chassis.pid_wait();
}


void progSkills79Worlds(){

  // setAntiJamActiveWorlds(true);

  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 20);

  // // get quartet
   
  // chassis.pid_drive_set(30.5_in, DRIVE_SPEED, true);  // moves towards triple balls
  // chassis.pid_wait_until(15_in);
  // chassis.pid_speed_max_set(DRIVE_SPEED*0.75);
  // matchloaderActive(true);
  // chassis.pid_wait();

  // // matchloaderInactive(true);

  // chassis.pid_turn_set(-88_deg, TURN_SPEED, false); // turn to middle goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(-19.75_in, DRIVE_SPEED, true);  // moves towards middle goal
  // chassis.pid_wait();

  // midDescore.retract();


  // turnScoringatXSpeed(true, -127*0.9);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // pros::delay(1500);

  // turnScoringatXSpeed(true, -127*0.5);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // pros::delay(500);

  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);




  // chassis.pid_drive_set(51.5_in, DRIVE_SPEED*0.9, true);  // lurch to matchload area
  // chassis.pid_wait();


  // midDescore.extend();
  
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // matchloaderActive(true);

  // chassis.pid_turn_set(-47.5_deg, TURN_SPEED, false); // turn to matchloader
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(14_in, DRIVE_SPEED*0.35, true); // drive towards matchloader
  // chassis.pid_wait();
  // pros::delay(1200);
  
  // chassis.pid_drive_set(-9.75_in, DRIVE_SPEED, true); // drive away from matchloader
  // chassis.pid_wait();
  // matchloaderInactive(true);
  
  // chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to alley
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // turnScoringatXSpeed(true, 0);
  // turnLoweratXSpeed(true, 0);
  // turnMiddleatXSpeed(true, 0);

  // chassis.pid_drive_set(13.5_in, DRIVE_SPEED, true); // drive towards alley
  // chassis.pid_wait();

  // chassis.drive_set(DRIVE_SPEED*0.5, DRIVE_SPEED*0.3); // curve into the alley
  // pros::delay(750);
  // chassis.drive_set(0, 0);


  // midDescore.retract();


  // chassis.drive_set(DRIVE_SPEED*0.95, DRIVE_SPEED); // drive across alley
  // pros::delay(800);
  // chassis.drive_set(DRIVE_SPEED*0.65, DRIVE_SPEED*0.7); // drive across alley
  // pros::delay(125);
  // chassis.drive_set(DRIVE_SPEED*0.35, DRIVE_SPEED*0.4); // drive across alley
  // pros::delay(75);
  // chassis.drive_set(DRIVE_SPEED*0.0, DRIVE_SPEED*0.0); // drive across alley
  // chassis.drive_set(0, 0);
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(500);

  // midDescore.extend();

  // // add distance sensor check here


  // // if(frontD.get() > 655 || frontD.get() < 645){
  // //   float DistanceToMove = frontD.get() - 650;
  // //   DistanceToMove *= 0.03937008;
  // //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
  // //   chassis.pid_wait();
  // // }


  
  // //turn is 135 bc that makes it 90 when hitting the wall
  // chassis.pid_turn_set(88_deg, TURN_SPEED, false); // turn out of alley
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(-8_in, 127, true); // wall check 
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // setAntiJamActiveWorlds(false);

  // chassis.pid_drive_set(16_in, DRIVE_SPEED, true); // drive infront of goal
  // chassis.pid_wait();

  // pros::delay(50);

  // chassis.pid_turn_set(-88_deg, TURN_SPEED, false); // turn to goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // matchloaderActive(true);

  // chassis.pid_drive_set(-12.5_in, DRIVE_SPEED*0.90, true); // drive to goal
  // chassis.pid_wait();

  // setAntiJamActiveWorlds(true);

  

  // matchloaderActive(true);

  // turnScoringatXSpeed(true, -127);
  // turnLoweratXSpeed(true, -127);
  // turnMiddleatXSpeed(true, -127);

  // pros::delay(100);
  
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // hood.extend();

  // pros::delay(1200);

  // hood.retract();

  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(31_in, DRIVE_SPEED*0.75, true); // drive into 2nd matchloader
  // chassis.pid_wait_until(10.5_in);
  // hood.retract();
  // turnScoringatXSpeed(true, -60);
  // turnLoweratXSpeed(true, 40);
  // turnMiddleatXSpeed(true, 40);
  // chassis.pid_wait_until(20_in);
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);
  // chassis.pid_speed_max_set(DRIVE_SPEED*0.35);
  // chassis.pid_wait();

  // pros::delay(1100);

  // chassis.pid_drive_set(-30.75_in, DRIVE_SPEED*0.9, true); // drive into goal for second score
  // chassis.pid_wait();
  

  // matchloaderActive(true);

  // turnScoringatXSpeed(true, -127);
  // turnLoweratXSpeed(true, -127);
  // turnMiddleatXSpeed(true, -127);

  // pros::delay(100);
  
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // hood.extend();

  // matchloaderInactive(true);

  // pros::delay(1300);


  // chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // hood.retract();

  // chassis.pid_drive_set(-5.5_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // chassis.pid_drive_set(15_in,DRIVE_SPEED*0.7,true); // drive away from goal
  // chassis.pid_wait();

  // chassis.pid_turn_set(90_deg, TURN_SPEED, false); //turn to other side
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(95.5_in,DRIVE_SPEED*0.7,true); // cross field
  // hood.retract();
  // turnScoringatXSpeed(true, -127);
  // turnLoweratXSpeed(true, 40);
  // turnMiddleatXSpeed(true, 40);
  // chassis.pid_wait_until(90_in);
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);
  // midDescore.retract();
  // chassis.pid_wait();

  // midDescore.extend();

  // chassis.pid_turn_set(-89_deg, TURN_SPEED, false); //turn to face goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // matchloaderActive(true);

  // chassis.pid_drive_set(-16.5_in,DRIVE_SPEED*0.75,true); // align with goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);




  // chassis.pid_drive_set(31.75_in, DRIVE_SPEED*0.6, true); // drive into 3rd matchloader
  // chassis.pid_wait_until(20_in);
  // chassis.pid_speed_max_set(DRIVE_SPEED*0.35);
  // chassis.pid_wait();
  
  // pros::delay(1000);


  // chassis.pid_drive_set(-9.75_in, DRIVE_SPEED, true); // drive away from matchloader
  // chassis.pid_wait();
  // matchloaderInactive(true);
  
  // chassis.pid_turn_set(135_deg, TURN_SPEED, false); // turn to alley
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  
  // turnScoringatXSpeed(true, 0);
  // turnLoweratXSpeed(true, 0);
  // turnMiddleatXSpeed(true, 0);


  // chassis.pid_drive_set(13.5_in, DRIVE_SPEED, true); // drive towards alley
  // chassis.pid_wait();

  // chassis.drive_set(DRIVE_SPEED*0.5, DRIVE_SPEED*0.3); // curve into the alley
  // pros::delay(750);
  // chassis.drive_set(0, 0);

  // midDescore.retract();


  // chassis.drive_set(DRIVE_SPEED*0.95, DRIVE_SPEED); // drive across alley
  // pros::delay(800);
  // chassis.drive_set(DRIVE_SPEED*0.65, DRIVE_SPEED*0.7); // drive across alley
  // pros::delay(125);
  // chassis.drive_set(DRIVE_SPEED*0.35, DRIVE_SPEED*0.4); // drive across alley
  // pros::delay(75);
  // chassis.drive_set(DRIVE_SPEED*0.0, DRIVE_SPEED*0.0); // drive across alley
  // chassis.drive_set(0, 0);
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // pros::delay(500);

  // midDescore.extend();


  // // add distance sensor check here


  // // if(frontD.get() > 655 || frontD.get() < 645){
  // //   float DistanceToMove = frontD.get() - 650;
  // //   DistanceToMove *= 0.03937008;
  // //   chassis.pid_drive_set(DistanceToMove, DRIVE_SPEED*0.4, true);
  // //   chassis.pid_wait();
  // // }


  
  // //turn is 135 bc that makes it 90 when hitting the wall
  // chassis.pid_turn_set(88_deg, TURN_SPEED, false); // turn out of alley
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(-8_in, 127, true); // wall check
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // setAntiJamActiveWorlds(false);

  // chassis.pid_drive_set(16.5_in, DRIVE_SPEED, true); // drive infront of goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_turn_set(-88_deg, TURN_SPEED, false); // turn to goal
  // chassis.pid_wait();
  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // matchloaderActive(true);

  // chassis.pid_drive_set(-15.5_in, DRIVE_SPEED*0.90, true); // drive to goal
  // chassis.pid_wait();

  // setAntiJamActiveWorlds(true);
  
  // turnScoringatXSpeed(true, -127);
  // turnLoweratXSpeed(true, -127);
  // turnMiddleatXSpeed(true, -127);

  // pros::delay(100);

  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // hood.extend();

  // pros::delay(1200);

  // chassis.odom_xyt_set(0_in, 0_in, 0_deg);

  // chassis.pid_drive_set(30.75_in, DRIVE_SPEED*0.75, true); // drive into 4th matchloader
  // chassis.pid_wait_until(10_in);
  // hood.retract();
  // turnScoringatXSpeed(true, -60);
  // turnLoweratXSpeed(true, 40);
  // turnMiddleatXSpeed(true, 40);
  // chassis.pid_wait_until(20_in);
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);
  // chassis.pid_speed_max_set(DRIVE_SPEED*0.35);
  // chassis.pid_wait();

  // pros::delay(1100);

  // chassis.pid_drive_set(-29_in, DRIVE_SPEED*0.9, true); // drive into goal for second score
  // chassis.pid_wait();

  // matchloaderActive(true);

  
  // turnScoringatXSpeed(true, -127);
  // turnLoweratXSpeed(true, -127);
  // turnMiddleatXSpeed(true, -127);

  // pros::delay(100);
  
  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // hood.extend();

  // matchloaderInactive(true);

  // pros::delay(1200);

  // turnScoringatXSpeed(true, -127);
  // turnLoweratXSpeed(true, -127);
  // turnMiddleatXSpeed(true, -127);

  // pros::delay(50);

  // turnScoringatXSpeed(true, 127);
  // turnLoweratXSpeed(true, 127);
  // turnMiddleatXSpeed(true, 127);

  // pros::delay(200);

  // chassis.pid_drive_set(5_in, DRIVE_SPEED, true);
  // chassis.pid_wait();

  // hood.retract();

  // chassis.pid_drive_set(-5.5_in, DRIVE_SPEED, true);
  // chassis.pid_wait();



  chassis.pid_drive_set(11_in, DRIVE_SPEED, true);
  chassis.pid_wait();
  chassis.odom_xyt_set(0_in, 11_in, 0_deg);

  pros::delay(50);

  chassis.odom_boomerang_dlead_set(0.3);

  

  chassis.pid_odom_set({{26.5_in, 34.75_in, 90_deg}, fwd, 83});
  chassis.pid_wait();

  turnScoringatXSpeed(true, -127);
  turnLoweratXSpeed(true, -127);
  turnMiddleatXSpeed(true, -127);


  chassis.pid_drive_set(47_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  turnScoringatXSpeed(true, 127);
  turnLoweratXSpeed(true, 127);
  turnMiddleatXSpeed(true, 127);

  chassis.pid_drive_set(-5_in, DRIVE_SPEED, true);
  chassis.pid_wait();

  

}


void testing(){
  
  
}