#pragma once

#include "main.h"

// ===== SUBSYSTEM DECLARATIONS =====
extern ez::Drive chassis;
extern Intake intake;

// ===== PNEUMATICS =====
// Four individual pistons for testing
inline ez::Piston piston1('E', false);  // Test with button X
inline ez::Piston piston2('F', false);  // Test with button B
inline ez::Piston piston3('G', false);  // Test with button Y
inline ez::Piston piston4('H', false);  // Test with button A
