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
  chassis.pid_drive_constants_set(5.0, 0.0, 40.0);
  chassis.pid_heading_constants_set(0.0, 0.0, 0.0);
  chassis.pid_turn_constants_set(2.0, 0.0, 10.0, 15);
  chassis.pid_swing_constants_set(5.0, 0.0, 30.0);
  chassis.pid_odom_angular_constants_set(6.0, 0.0, 50.0);
  chassis.pid_odom_boomerang_constants_set(5.0, 0.0, 45.0);

  // Exit conditions - REDUCED TIMEOUTS FOR SPEED
  chassis.pid_turn_exit_condition_set(50_ms, 2_deg, 150_ms, 5_deg, 250_ms, 250_ms);
  chassis.pid_swing_exit_condition_set(50_ms, 2_deg, 150_ms, 5_deg, 250_ms, 250_ms);
  chassis.pid_drive_exit_condition_set(10_ms, 1_in, 100_ms, 2_in, 200_ms, 200_ms);
  chassis.pid_odom_turn_exit_condition_set(50_ms, 2_deg, 150_ms, 5_deg, 250_ms, 500_ms);
  chassis.pid_odom_drive_exit_condition_set(50_ms, 1_in, 150_ms, 2_in, 250_ms, 500_ms);
  
  // Chain constants - TIGHTER FOR SMOOTHER FLOW
  chassis.pid_turn_chain_constant_set(5_deg);
  chassis.pid_swing_chain_constant_set(7_deg);
  chassis.pid_drive_chain_constant_set(5_in);

  // Slew constants
  chassis.slew_turn_constants_set(3_deg, 70);
  chassis.slew_drive_constants_set(3_in, 50);
  chassis.slew_swing_constants_set(3_in, 60);

  chassis.odom_turn_bias_set(0.95);
  chassis.odom_look_ahead_set(7_in);
  chassis.odom_boomerang_distance_set(16_in);
  chassis.odom_boomerang_dlead_set(0.625);
  chassis.pid_angle_behavior_set(ez::shortest);
}

// Autonoous Routines

/*
 *
 * towards the enemy = 0 degrees, positive Y
 * towards the driver = 180 deg, negative Y
 * driver's right = 90 deg, positive X
 * driver's left = -90 deg / 270 deg, negative X
 * 
 */

// does long goal on right side
// offset can be used if starting position is farther away from the parking zone, in the first motion of going to the matchloader
void long_goal_base(okapi::QLength extraOffset = 0_in) {
  // 1. move 42.5 inches forward
  chassis.pid_drive_set(42.4_in, DRIVE_SPEED * 0.9);
  chassis.pid_wait_quick();

  // 2. turn towards driver
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 3. activate little will mech
  tongue_piston.set(true);

  pros::delay(500);

  // 4. move 15 inches forward, and extra offset
  chassis.pid_drive_set(14.9_in + extraOffset, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake for 1 second
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(800);

  // 6. move in reverse 41.5 inches
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-42.25_in, DRIVE_SPEED);

  // 7. deactivate little will mech while moving
  tongue_piston.set(false);

  // 8. once you finish moving 33 inches run high goal for 2.5 seconds
  chassis.pid_wait_quick();
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2500);
  scoring.set_state_and_move(Scoring::State::NONE);
}

// long goal right side + middle low
void right_side_auton() {
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);

  long_goal_base();

  // 9. move forward 19.5 inches
  chassis.pid_drive_set(28_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 10. move 135 degrees to the right
  chassis.pid_turn_relative_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 11. move 60 inches forward
  chassis.pid_drive_set(69_in, DRIVE_SPEED);

  // 12. keep intaking until you reach 54 inches of the 60
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_wait_until(54_in);

  // 13. for the next 6 inches run reverse intake
  scoring.set_state_and_move(Scoring::State::OUTTAKING);
  pros::delay(1000);
  scoring.set_state_and_move(Scoring::State::NONE);
}

// long goal left side + middle high
void left_side_auton() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);

  long_goal_base();

  // 9. move forward 20 inches
  chassis.pid_drive_set(30_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 10. move 135 degrees to the left
  chassis.pid_turn_relative_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 11. move 56.5 inches forward while intaking
  chassis.pid_drive_set(63.5_in, DRIVE_SPEED * 0.6);
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_wait();
  scoring.set_state_and_move(Scoring::State::NONE);

  // 12. turn 180 degrees
  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  // 13. move 3.75 inches in reverse
  chassis.pid_drive_set(-9_in, DRIVE_SPEED);
  chassis.pid_wait();

  // 14. run high goal for 1.5 seconds
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(1500);
  scoring.set_state_and_move(Scoring::State::NONE);
}

// comment because this function felt lonely without one
void skills_auton() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);

  long_goal_base();

  // away from goal
  chassis.pid_drive_set(15_in, DRIVE_SPEED);
  chassis.pid_wait();

  // turn to right side
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(3.1_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // right side
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);

  long_goal_base(7_in); // use extra offset

  // then do middle goals or something

  // fefefe;fmekfmeqlkfmwel
}
