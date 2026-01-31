#pragma once

#include "EZ-Template/api.hpp"

class Scoring;

// Subsystem Declarations
extern ez::Drive chassis;
extern Scoring scoring;

// Pneumatics
inline ez::Piston tongue_piston('G', false);
inline ez::Piston wing_piston('H', false);
