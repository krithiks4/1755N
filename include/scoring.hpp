// Scoring Mechanism
// Top Motor (12) and Bottom Motor (11)

#pragma once

#include "api.h"

class Scoring {
    public:
        enum class State {
            NONE,
            INTAKING,       // Both motors forward
            OUTTAKING,      // Both motors reverse
            HIGH_GOAL,      // Top reverse, bottom forward
            MIDDLE_GOAL,    // Top reverse, bottom forward
            STORAGE,        // Only bottom forward
            LOW_GOAL        // Only bottom forward
        };

        pros::Motor top_motor;
        pros::Motor bottom_motor;
    private:
        State state = State::NONE;

        void move();
    public:
        Scoring(int8_t top_port, int8_t bottom_port)
            : top_motor(top_port), bottom_motor(bottom_port)
        {
            top_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
            bottom_motor.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        }

        void init() {
            // Call this during initialize() after PROS is ready
            top_motor.set_gearing(pros::E_MOTOR_GEAR_200);
            bottom_motor.set_gearing(pros::E_MOTOR_GEAR_200);
        }

        void set_state(State state);
        void set_state_and_move(State state);

        void opcontrol(pros::Controller& controller);
};
