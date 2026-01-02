#pragma once

#include "main.h"

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
// Middle goal piston (H) and Tongue mech piston (G)
inline ez::Piston middle_goal_piston('H', true);  // Middle goal piston
inline ez::Piston tongue_piston('G', false);  // Tongue mech piston
