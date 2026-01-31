#include "autons.hpp"

// PID Constants
void default_constants() {
  
  // P, I, D, and Start I
  chassis.pid_drive_constants_forward_set(7.0, 0.0, 65.0);
  chassis.pid_drive_constants_backward_set(6.5, 0.0, 53.0);
  chassis.pid_heading_constants_set(4.0, 0.0, 6.0);
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

/*
 * for Autonoous Routines

 * towards the enemy = 0 degrees, positive Y
 * towards the driver = 180 deg, negative Y
 * driver's right = 90 deg, positive X
 * driver's left = -90 deg / 270 deg, negative X
 */

 void test_auton() {
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_odom_set({{0_in, 20_in}, FORWARD, 50});
  chassis.pid_wait();
 }