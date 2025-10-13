#include "subsystems.hpp"

// Default ports (override in main.cpp or here)
#ifndef MOTOR_HIGH_GOAL_PORT
#define MOTOR_HIGH_GOAL_PORT 1
#endif
#ifndef MOTOR_MIDDLE_GOAL_PORT
#define MOTOR_MIDDLE_GOAL_PORT 2
#endif

#ifndef MOTOR_INTAKE_PORT
#define MOTOR_INTAKE_PORT 3
#endif
#ifndef MOTOR_INDEXER_PORT
#define MOTOR_INDEXER_PORT 11
#endif

// Default velocities matching PROS defaults (green cartridges -> 200 rpm)
static constexpr int DEFAULT_GEARSET_GREEN_VEL = 200;
static constexpr int DEFAULT_SMART_MOTOR_VEL = 100; // half power for smart motors
static constexpr int DEFAULT_GEARSET_BLUE_VEL = 600; // if you use blue

// Motors (defined in the subsystems namespace so externs in the header link)
pros::Motor subsystems::high_goal_motor(MOTOR_HIGH_GOAL_PORT);
pros::Motor subsystems::middle_goal_motor(MOTOR_MIDDLE_GOAL_PORT);
pros::Motor subsystems::intake_motor(MOTOR_INTAKE_PORT);
pros::Motor subsystems::indexer_motor(MOTOR_INDEXER_PORT);

namespace subsystems {

void initialize() {
  // Configure brakes and ensure stopped
  high_goal_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  middle_goal_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  indexer_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  high_goal_motor.move_velocity(0);
  middle_goal_motor.move_velocity(0);
  intake_motor.move_velocity(0);
  indexer_motor.move_velocity(0);
}

void spin_high_goal(int velocity) {
  high_goal_motor.move_velocity(velocity);
}

void spin_middle_goal(int velocity) {
  middle_goal_motor.move_velocity(velocity);
}

void spin_intake(int velocity) {
  intake_motor.move_velocity(velocity);
}

void spin_indexer(int velocity) {
  indexer_motor.move_velocity(velocity);
}

void update_opcontrol() {
  // Buttons mapping (override in main if desired)
  // High goal: R1 -> clockwise
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    spin_high_goal(DEFAULT_GEARSET_GREEN_VEL);
  } else {
    spin_high_goal(0);
  }

  // Middle goal: R2 -> counterclockwise
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    spin_middle_goal(-DEFAULT_GEARSET_GREEN_VEL);
  } else {
    spin_middle_goal(0);
  }

  // Intake: L1 forward, L2 reverse (half power)
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    spin_intake(DEFAULT_SMART_MOTOR_VEL);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    spin_intake(-DEFAULT_SMART_MOTOR_VEL);
  } else {
    spin_intake(0);
  }

  // Indexer: Up forward, Down reverse (half power)
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    spin_indexer(DEFAULT_SMART_MOTOR_VEL);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    spin_indexer(-DEFAULT_SMART_MOTOR_VEL);
  } else {
    spin_indexer(0);
  }
}

} // namespace subsystems
