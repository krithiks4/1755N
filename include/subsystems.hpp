#pragma once

#include "main.h"

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
// Middle goal piston (H) and Tongue mech piston (G)
inline ez::Piston piston1('H', true);  // Middle goal piston
inline ez::Piston piston2('G', true);  // Tongue mech piston
