#include "main.h"
#include "scoring.hpp"

void Scoring::move() {
    switch (state) {
        case State::INTAKING:
        case State::HIGH_GOAL:
        case State::MIDDLE_GOAL:
            // Both motors spinning forward
            top_motor.move_voltage(12000);     // 12V forward
            bottom_motor.move_voltage(12000);  // 12V forward
            break;
        case State::OUTTAKING:
        case State::STORAGE:
        case State::LOW_GOAL:
            // Only bottom motor going forward
            top_motor.brake();
            bottom_motor.move_voltage(12000);  // 12V forward
            break;
        case State::NONE:
        default:
            top_motor.brake();
            bottom_motor.brake();
            break;
    }
}

void Scoring::set_state(State state) {
    this->state = state;
}

void Scoring::set_state_and_move(State state) {
    this->state = state;
    move();
}

void Scoring::opcontrol(pros::Controller& controller, bool piston_h_state) {
    if (controller.get_digital(DIGITAL_R1)) {
        set_state(State::INTAKING);
    } else if (controller.get_digital(DIGITAL_L1)) {
        if (piston_h_state) {
            set_state(State::MIDDLE_GOAL);
        } else {
            set_state(State::HIGH_GOAL);
        }
    } else if (controller.get_digital(DIGITAL_L2)) {
        set_state(State::LOW_GOAL);
    } else {
        set_state(State::NONE);
    }

    move();
}
