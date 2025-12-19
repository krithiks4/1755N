/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"
#include "pros/motor_group.hpp"

class Intake {
    public:
        enum class State {
            NONE,
            INTAKING,
            OUTTAKING
        };

        pros::MotorGroup intake_motors;
    private:
        State state = State::NONE;

        const int INTAKE_VOLTAGE = 127;

        void move();
    public:
        Intake(std::initializer_list<std::int8_t> intake_ports)
            : intake_motors(intake_ports)
        {
            intake_motors.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }

        void set_state(State state);
        void set_state_and_move(State state);

        void opcontrol();
};
