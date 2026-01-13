#pragma once

#include "main.h"

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
// Tongue mech piston (G)
inline ez::Piston tongue_piston('G', false);  // Tongue mech piston
