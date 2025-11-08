#include "main.h"
#include "autons.hpp"

// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 40;
const int SWING_SPEED = 110;

///
// Constants
///
void default_constants() {
  // P, I, D, and Start I
  chassis.pid_drive_constants_set(8.0, 2.30, 7.0);         // Fwd/rev constants, used for odom and non odom motions
  chassis.pid_heading_constants_set(6.0, 1.0, 8.0);        // Holds the robot straight while going forward without odom
  chassis.pid_turn_constants_set(6.0, 2.0, 8.0, 0);     // Turn in place constants
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

void pid_tuning_test() {
  // Drive 24 inches (2 feet) forward for PID tuning
  chassis.pid_turn_set(90_deg, DRIVE_SPEED);
  chassis.pid_wait();
}

static void flipThetaIfNotAlreadyFlipped() {
  if (!chassis.odom_theta_direction_get()) chassis.odom_theta_flip();
}

// longGoalRight
static void longGoalBase() {
  chassis.pid_drive_set(38.5_in, DRIVE_SPEED);
  chassis.pid_wait();

  // will.set(true);
  chassis.pid_turn_relative_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  // sprockets.set_state_and_move(Sprockets::State::INTAKE);

  // chassis.pid_drive_set(8_in, DRIVE_SPEED);
  // chassis.pid_wait();

  // for (int i = 0; i < 3; i++)
  // {
  //   chassis.pid_drive_set(7_in, 127);
  //   pros::delay(300);
  //   chassis.pid_drive_set(-7_in, 127);
  //   pros::delay(300);
  // }
  // chassis.pid_drive_set(0_in, DRIVE_SPEED);

  // sprockets.set_state_and_move(Sprockets::State::NONE);

  chassis.pid_drive_set(-32_in, DRIVE_SPEED);
  chassis.pid_wait();

  // will.set(false);

  sprockets.set_state_and_move(Sprockets::State::HIGH_GOAL);
  pros::delay(2000);
  sprockets.set_state_and_move(Sprockets::State::NONE);
}

// preload + matchload -> long goal on right side
void longGoalRight() {
  longGoalBase();
}

// preload + matchload -> long goal on left side
void longGoalLeft() {
  flipThetaIfNotAlreadyFlipped();
  longGoalBase();
}

// rightGoals, but without ending
static void sideGoalsBase() {
  chassis.pid_drive_set(12_in, DRIVE_SPEED);
  chassis.pid_wait();

  chassis.pid_turn_relative_set(310_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  sprockets.set_state_and_move(Sprockets::State::INTAKE);

  chassis.pid_drive_set(32_in, DRIVE_SPEED);
  chassis.pid_wait_quick();
  pros::delay(400);
  sprockets.set_state_and_move(Sprockets::State::NONE);
}

// preload+matchload -> long goal on right side -> pick up middle 3 -> low goal
void rightGoals() {
  longGoalRight(); 
  sideGoalsBase();

  chassis.pid_turn_relative_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  chassis.pid_drive_set(-24.1_in, DRIVE_SPEED);
  chassis.pid_wait();

  sprockets.set_state_and_move(Sprockets::State::MIDDLE_GOAL);
  pros::delay(3000);
  sprockets.set_state_and_move(Sprockets::State::NONE);
}

// preload+matchload -> long goal on left side -> pick up middle 3 -> middle goal
void leftGoals() {
  flipThetaIfNotAlreadyFlipped();
  longGoalLeft();
  sideGoalsBase();

  sprockets.set_state_and_move(Sprockets::State::INTAKE);

  chassis.pid_drive_set(-24.1_in, DRIVE_SPEED);
  chassis.pid_wait();

  sprockets.set_state_and_move(Sprockets::State::LOW_GOAL);
  pros::delay(3000);
  sprockets.set_state_and_move(Sprockets::State::NONE);
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
