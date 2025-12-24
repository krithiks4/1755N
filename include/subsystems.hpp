#pragma once

#include "main.h"

// ===== SUBSYSTEM DECLARATIONS =====
extern ez::Drive chassis;
extern Intake intake;

// ===== PNEUMATICS =====
// Four individual pistons
inline ez::Piston intake_lift('E', false); 
inline ez::Piston indexer('F', false);     \
inline ez::Piston wing('G', false);        
inline ez::Piston little_will_mech('H', false);  
