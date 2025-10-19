// subsystems.hpp
#pragma once

#include "main.h"

namespace subsystems {
void initialize();
void update_opcontrol();
void spin_high_goal(int velocity);
void spin_intake(int velocity);
void spin_indexer(int velocity);
extern pros::Motor high_goal_motor;
extern pros::Motor intake_motor;
extern pros::Motor indexer_motor;
extern pros::adi::DigitalOut wall_mech;
}  // namespace subsystems
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples

// inline pros::Motor intake(1);
// inline pros::adi::DigitalIn limit_switch('A');