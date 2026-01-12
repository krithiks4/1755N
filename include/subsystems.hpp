#pragma once

#include "main.h"

// ===== SUBSYSTEM DECLARATIONS =====
extern ez::Drive chassis;
extern Intake intake;

// ===== PNEUMATICS =====
// Four individual pistons
inline ez::Piston double_park('C', false);   
inline ez::Piston wing('B', false);        
inline ez::Piston lil_krith('A', false);  
