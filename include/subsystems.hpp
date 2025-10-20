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
extern ez::Piston wall_mech;
}  // namespace subsystems
#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;