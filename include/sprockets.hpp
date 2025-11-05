/*
This Source Code Form is subject to the terms of the Mozilla Public
License, v. 2.0. If a copy of the MPL was not distributed with this
file, You can obtain one at http://mozilla.org/MPL/2.0/.
*/

#pragma once

#include "api.h"
#include "pros/motors.h"

class Sprockets {
    public:
        enum class State {
            NONE,
            INTAKE,
            LOW_GOAL,
            MIDDLE_GOAL,
            HIGH_GOAL
        };

        pros::Motor intake_motor;
        pros::Motor low_motor;
        pros::Motor middle_motor;
        pros::Motor high_motor;
    private:
        State state = State::NONE;

        const int SPROCKET_VOLTAGE = 127;

        void move();
    public:
        Sprockets(int intake_port, int low_port, int middle_port, int high_port)
            : intake_motor(intake_port), low_motor(low_port), middle_motor(middle_port), high_motor(high_port)
        {
            intake_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            low_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            middle_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            high_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }

        void set_state(State state);
        void set_state_and_move(State state);

        void opcontrol();
};
