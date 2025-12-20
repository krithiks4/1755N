#pragma once

#include "main.h"

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
// Middle goal piston (A) and Tongue mech piston (B)
inline ez::Piston piston1('A', false);  // Middle goal piston
inline ez::Piston piston2('B', false);  // Tongue mech piston
