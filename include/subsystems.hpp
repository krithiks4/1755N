#pragma once

#include "main.h"

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
// Middle goal piston (H) and Tongue mech piston (G)
inline ez::Piston middleGoalPiston('H', false);  // Middle goal piston
inline ez::Piston tonguePiston('G', true);  // Tongue mech piston
