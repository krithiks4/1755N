#include "main.h"
#include "autons.hpp"

// Autonomous Constants
// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

// PID Constants
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(5.0, 0.0, 70.0);        // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(11.0, 0.0, 20.0);      // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(2.0, 0.0, 10.0, 15);      // Turn in place constants
  chassis.pid_swing_constants_set(5.0, 0.0, 30.0);         // Swing constants
  chassis.pid_odom_angular_constants_set(6.0, 0.0, 50.0);  // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(5.0, 0.0, 45.0); // Angular control for boomerang motions

  // Exit conditions
  chassis.pid_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_swing_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 500_ms);
  chassis.pid_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 500_ms);
  chassis.pid_odom_turn_exit_condition_set(90_ms, 3_deg, 250_ms, 7_deg, 500_ms, 750_ms);
  chassis.pid_odom_drive_exit_condition_set(90_ms, 1_in, 250_ms, 3_in, 500_ms, 750_ms);
  chassis.pid_turn_chain_constant_set(3_deg);
  chassis.pid_swing_chain_constant_set(5_deg);
  chassis.pid_drive_chain_constant_set(3_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 50);
  chassis.slew_swing_constants_set(3_in, 60);

  // The amount that turns are prioritized over driving in odom motions
  // - with tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.95);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// Autonomous Routines
void pid_tuning_test() {
  // Drive 24 inches (2 feet) forward for PID tuning
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void drive_test() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  chassis.pid_wait();
}

void turn_test() {
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();
  pros::delay(1000);
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

void intake_test() {
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::MIDDLE_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::STORAGE);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);
}

void odom_test() {
  // Reset odometry position
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  // Simple odometry test - drive forward and display position
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(1000);
  
  // Turn 90 degrees and drive again
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(1000);
  
  // Return to start
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_drive_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}