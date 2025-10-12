// subsystems.hpp
#pragma once

#include "main.h"

namespace subsystems {
// Call during robot initialize()
void initialize();

// Call from opcontrol loop each iteration to handle button inputs
void update_opcontrol();

// If you want to command these directly from other code, these helper APIs are provided
void spin_high_goal(int velocity);
void spin_middle_goal(int velocity);
void spin_intake(int velocity);
void spin_indexer(int velocity);

// Motors (externs) - you can use these directly if needed. Definitions live in subsystems.cpp
extern pros::Motor high_goal_motor;
extern pros::Motor middle_goal_motor;
extern pros::Motor intake_motor_a;
extern pros::Motor intake_motor_b;
extern pros::Motor indexer_motor_a;
extern pros::Motor indexer_motor_b;
}  // namespace subsystems
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');