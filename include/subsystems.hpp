#pragma once

#include "main.h"

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
// Middle goal piston (G) and Tongue mech piston (H)
inline ez::Piston piston1('G', false);  // Middle goal piston
inline ez::Piston piston2('H', false);  // Tongue mech piston
