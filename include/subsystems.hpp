#pragma once

#include "main.h"

// ===== SUBSYSTEM DECLARATIONS =====
extern ez::Drive chassis;
extern Intake intake;

// ===== PNEUMATICS =====
// Three individual pistons
inline ez::Piston piston1('A', false);
inline ez::Piston piston2('B', false);
inline ez::Piston piston3('C', false);
