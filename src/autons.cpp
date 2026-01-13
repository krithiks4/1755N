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

// long goal right side + middle low
void right_side_auton() {
  chassis.odom_xyt_set(0_in, 0_in, 90_deg); // STARTING POSITION: ROTATION: FACE RIGHT, X: EXACTLY MIDDLE OF FIELD, Y: RIGHT SIDE TOUCHING PARKING ZONE
  
  // go forward one tile
  chassis.pid_odom_set({{1_tile, 0_in}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  scoring.set_state_and_move(Scoring::State::INTAKING);

  // go to center balls
  chassis.pid_odom_set({{1_tile, 1_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait_quick_chain();

  // go to center goal
  chassis.pid_odom_set({{0.5_tile, 1.5_tile, -45_deg}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // low goal
  scoring.set_state_and_move(Scoring::State::OUTTAKING);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader (activate tonge)
  chassis.pid_odom_set({{2_tile, 0_tile}, FORWARD, DRIVE_SPEED});
  tongue_piston.set(true);
  chassis.pid_wait();

  // 2. turn towards matchloader
  chassis.pid_turn_set({2_tile, -1_tile}, FORWARD, TURN_SPEED);
  chassis.pid_wait_quick();

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(14.9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(450);

  // 6. move to long goal (back some offset)
  chassis.pid_odom_set({{2_tile, 1_tile - 10_in}, REVERSE, DRIVE_SPEED});
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader
  chassis.pid_odom_set({{2_tile, 0_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // go to right side of goal so that wing can be used
  chassis.pid_odom_set({{2.5_tile, 1.4_tile, 0_deg}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // go backwards to stop touching the balls????
}

// long goal left side + middle high
void left_side_auton() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
}

// comment because this function felt lonely without one
void skills_auton() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);
}
