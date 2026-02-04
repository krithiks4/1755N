#pragma once

#include "main.h"
#include "EZ-Template/api.hpp"
#include "subsystems.hpp"

// Autonomous Constants
// These are out of 127
const int DRIVE_SPEED = 105;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

void default_constants();

void matchload_wiggle(int times = 6);
void highgoal_antijam(int times = 7);

void test_auton();
