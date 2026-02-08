#pragma once

#include "main.h"
#include "EZ-Template/api.hpp"
#include "subsystems.hpp"

void default_constants();

void matchload_wiggle(int times = 6);
void highgoal_antijam(int times = 7);

void test_auton();
void wing_auton(bool right);
