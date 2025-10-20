#include "subsystems.hpp"

#ifndef MOTOR_HIGH_GOAL_PORT
#define MOTOR_HIGH_GOAL_PORT 1
#endif

#ifndef MOTOR_INTAKE_PORT
#define MOTOR_INTAKE_PORT 3
#endif

#ifndef MOTOR_INDEXER_PORT
#define MOTOR_INDEXER_PORT 2
#endif

// Motor max velocities (RPM)
static constexpr int HIGH_GOAL_MAX_VEL = 200;
static constexpr int INTAKE_MAX_VEL = 200;
static constexpr int INDEXER_MAX_VEL = 200;

pros::Motor subsystems::high_goal_motor(MOTOR_HIGH_GOAL_PORT);
pros::Motor subsystems::intake_motor(MOTOR_INTAKE_PORT);
pros::Motor subsystems::indexer_motor(MOTOR_INDEXER_PORT);

ez::Piston subsystems::wall_mech('A', false);

namespace subsystems {

void initialize() {
  high_goal_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
  indexer_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);

  high_goal_motor.move_velocity(0);
  intake_motor.move_velocity(0);
  indexer_motor.move_velocity(0);
}

void spin_high_goal(int velocity) {
  high_goal_motor.move_velocity(velocity);
}

void spin_intake(int velocity) {
  intake_motor.move_velocity(velocity);
}

void spin_indexer(int velocity) {
  indexer_motor.move_velocity(velocity);
}

void update_opcontrol() {
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    spin_high_goal(HIGH_GOAL_MAX_VEL);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    spin_high_goal(-HIGH_GOAL_MAX_VEL);
  } else {
    spin_high_goal(0);
  }

  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
    spin_intake(INTAKE_MAX_VEL);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
    spin_intake(-INTAKE_MAX_VEL);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
    spin_intake(INTAKE_MAX_VEL);
  } else {
    spin_intake(0);
  }

  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
    spin_indexer(INDEXER_MAX_VEL);
  } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    spin_indexer(-INDEXER_MAX_VEL);
  } else {
    spin_indexer(0);
  }

  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
    spin_high_goal(HIGH_GOAL_MAX_VEL);
    spin_indexer(INDEXER_MAX_VEL);
  }
  
  if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
    spin_high_goal(-HIGH_GOAL_MAX_VEL);
    spin_indexer(INDEXER_MAX_VEL);
  }

  wall_mech.button_toggle(master.get_digital(pros::E_CONTROLLER_DIGITAL_Y));
}

}
