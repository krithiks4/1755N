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
  chassis.pid_drive_constants_set(5.0, 0.0, 70.0);
  chassis.pid_heading_constants_set(0.0, 0.0, 0.0);  
  chassis.pid_turn_constants_set(2.0, 0.0, 10.0, 15);
  chassis.pid_swing_constants_set(5.0, 0.0, 30.0);
  chassis.pid_odom_angular_constants_set(6.0, 0.0, 50.0);
  chassis.pid_odom_boomerang_constants_set(5.0, 0.0, 45.0);

  // Exit conditions - REDUCED TIMEOUTS FOR SPEED
  chassis.pid_turn_exit_condition_set(50_ms, 2_deg, 150_ms, 5_deg, 250_ms, 250_ms);
  chassis.pid_swing_exit_condition_set(50_ms, 2_deg, 150_ms, 5_deg, 250_ms, 250_ms);
  chassis.pid_drive_exit_condition_set(50_ms, 1_in, 150_ms, 2_in, 250_ms, 250_ms);
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

#pragma region tests
void pid_tuning_48_in() {
  // Drive 48 inches (4 feet) forward for PID tuning
  chassis.pid_drive_set(48_in, DRIVE_SPEED);
  chassis.pid_wait();
}

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
  chassis.pid_odom_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(1000);
  
  // Turn 90 degrees and drive again
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_odom_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  pros::delay(1000);
  
  // Return to start
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_odom_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(270_deg, TURN_SPEED);
  chassis.pid_wait();
  
  chassis.pid_odom_set(24_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();
}

void odom_test_points() {
  // Reset odometry position
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  
  chassis.pid_odom_pp_set({
    {{0_in, 24_in}},
    {{24_in, 24_in}},
    {{24_in, 0_in}},
    {{0_in, 0_in}}
  });
  chassis.pid_wait();
}
#pragma endregion tests

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
void long_goal_base() {
  // 1. move 42.5 inches forward
  chassis.pid_odom_set(42.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 2. turn towards driver
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 3. activate little will mech
  tongue_piston.set(true);

  // 4. move 9 inches forward
  chassis.pid_odom_set(9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake for 1 second
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(1000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // 6. move in reverse 41.5 inches
  chassis.pid_odom_set(-41.5_in, DRIVE_SPEED);

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
  chassis.pid_odom_set(19.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 10. move 135 degrees to the right
  chassis.pid_turn_relative_set(135_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 11. move 60 inches forward
  chassis.pid_odom_set(60_in, DRIVE_SPEED);

  // 12. keep intaking until you reach 54 inches of the 60
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_wait_until(54_in);

  // 13. for the next 6 inches run reverse intake
  scoring.set_state_and_move(Scoring::State::OUTTAKING);
  chassis.pid_wait();  // Keep regular wait for the final movement
  scoring.set_state_and_move(Scoring::State::NONE);
}

// long goal left side + middle high
void left_side_auton() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);

  long_goal_base();

  // 9. move forward 20 inches
  chassis.pid_odom_set(20_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 10. move 135 degrees to the left
  chassis.pid_turn_relative_set(-135_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 11. move 56.5 inches forward while intaking
  chassis.pid_odom_set(56.5_in, DRIVE_SPEED);
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_wait();
  scoring.set_state_and_move(Scoring::State::NONE);

  // 12. turn 180 degrees
  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  // 13. move 3.75 inches in reverse
  chassis.pid_odom_set(-3.75_in, DRIVE_SPEED);
  chassis.pid_wait();

  // 14. run high goal for 1.5 seconds
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(1500);
  scoring.set_state_and_move(Scoring::State::NONE);
}

// comment because this function felt lonely without one
void skills_auton() {
  skills_half();
  skills_half();

  chassis.odom_xyt_set(0_in, 0_in, 90_deg);

  // park, from the side of the parking square
  chassis.pid_odom_set(-15_in, DRIVE_SPEED);
  chassis.pid_wait();
  
  chassis.pid_turn_relative_set(30_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(25_in, DRIVE_SPEED); // no odom, start thrusting
  chassis.pid_wait_quick_chain();
}

// go from start, do both sides of Right long goal
void skills_half()
{
  chassis.odom_xyt_set(0_in, 0_in, 90_deg);
  long_goal_base();

  // go around the long goal and to the other side
  chassis.pid_odom_set(19.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_set(0_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_odom_set({-10_in, 19.5_in, 0_deg});
  chassis.pid_wait_quick();

  // pass to the other side
  chassis.pid_odom_set(2_tile, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // from the left side of the long goal to the match loader
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_odom_set({10_in, 2_tile - 9_in, 0_deg});
  chassis.pid_wait_quick();

  // 3. activate little will mech
  tongue_piston.set(true);

  // 4. move 9 inches forward
  chassis.pid_odom_set(9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake for 1 second
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(1000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // 6. move in reverse 41.5 inches
  chassis.pid_odom_set(-41.5_in, DRIVE_SPEED);

  // 7. deactivate little will mech while moving
  tongue_piston.set(false);

  // 8. once you finish moving 33 inches run high goal for 2.5 seconds
  chassis.pid_wait_quick();
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2500);
  scoring.set_state_and_move(Scoring::State::NONE);

  // return to position on the other side
  chassis.pid_odom_set(9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 2. turn towards left, towards the other side's parking
  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  // 1. move 42.5 inches forward
  chassis.pid_odom_set(42.5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
}