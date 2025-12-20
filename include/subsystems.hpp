#pragma once

#include "main.h"

// ===== SUBSYSTEM DECLARATIONS =====
extern ez::Drive chassis;
extern Scoring scoring;

// ===== PNEUMATICS =====
// Middle goal piston (A) and Tongue mech piston (B)
inline ez::Piston piston1('A', false);  // Middle goal piston
inline ez::Piston piston2('B', false);  // Tongue mech piston
