// subsystems.hpp
#pragma once

#include "main.h"
#include "EZ-Template/api.hpp"
#include "api.h"

extern Drive chassis;
extern Sprockets sprockets;

inline ez::Piston will_mech('A', false);
