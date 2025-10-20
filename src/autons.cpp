#include "main.h"
#include "autons.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(10.0, 0.0, 0.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(3.0, 0.0, 10.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(3.0, 0.0, 20.0, 0.01);     // Turn in place constants
  chassis.pid_swing_constants_set(5.0, 0.0, 30.0);           // Swing constants
  chassis.pid_odom_angular_constants_set(4.0, 0.0, 40.0);    // Angular control for odom motions
  chassis.pid_odom_boomerang_constants_set(4.0, 0.0, 30.0);  // Angular control for boomerang motions

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
  // - if you have tracking wheels, you can run this higher.  1.0 is the max
  chassis.odom_turn_bias_set(0.9);

  chassis.odom_look_ahead_set(7_in);           // This is how far ahead in the path the robot looks at
  chassis.odom_boomerang_distance_set(16_in);  // This sets the maximum distance away from target that the carrot point can be
  chassis.odom_boomerang_dlead_set(0.625);     // This handles how aggressive the end of boomerang motions are

  chassis.pid_angle_behavior_set(ez::shortest);  // Changes the default behavior for turning, this defaults it to the shortest path there
}

// preload + matchload -> long goal on right side
void longGoalRight() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, false, true);
  chassis.pid_wait();

  chassis.pid_turn_set(-90_deg, TURN_SPEED);
  chassis.pid_wait();

  subsystems::wall_mech.set(true);

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  subsystems::wall_mech.set(false);
  chassis.pid_wait();

  // TODO: Score high goal here
}

// preload + matchload -> long goal on left side
void longGoalLeft() {
  chassis.pid_drive_set(24_in, DRIVE_SPEED, false, true);
  chassis.pid_wait();

  chassis.pid_turn_set(90_deg, TURN_SPEED); // Change -90 to +90 for left side
  chassis.pid_wait();

  subsystems::wall_mech.set(true);

  chassis.pid_drive_set(5_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
  chassis.pid_drive_set(-24_in, DRIVE_SPEED);
  subsystems::wall_mech.set(false);
  chassis.pid_wait();

  // TODO: Score high goal here
}

// preload+matchload -> long goal on right side -> pick up middle 3 -> low goal
void rightGoals() {
  longGoalRight();

  // TODO: Proceed to do middle
}

// preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal
void leftGoals() {
  longGoalLeft();

  // TODO: Proceed to do middle
}

// preload+matchload -> long goal on right side -> pick up middle 3 -> low goal -> middle goal
void rightLongGoalFullMiddle() {
  rightGoals();

  // TODO: Proceed to do the other middle goal
}

// preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal -> low goal
void leftLongGoalFullMiddle() {
  leftGoals();

  // TODO: Proceed to do the other middle goal
}

// preload+matchload -> long goal on right side -> pick up middle 3 -> low goal -> pick up left side middle 3 -> left side long goal
void allGoalsRight() {
  rightGoals();

  // TODO: Proceed to take the other middle 3 blocks for the other long goal
}

// preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal -> pick up right side middle 3 -> right side long goal
void allGoalsLeft() {
  leftGoals();

  // TODO: Proceed to take the other middle 3 blocks for the other long goal
}
