#include "main.h"
#include "intake.hpp"

void Intake::move() {
    switch (state) {
        case State::INTAKING:
            intake_motors.move_voltage(12000);  // 12V forward
            break;
        case State::OUTTAKING:
            intake_motors.move_voltage(-12000);  // 12V reverse
            break;
        case State::MIDDLE:  
            intake_motors.move_voltage(12000);  // 12V reverse
            scoring_motors.move_voltage(7000);
            break;
        case State::NONE:
        default:
            intake_motors.brake();
            break;
        case State::DP:
            intake_motors.move_voltage(-12000);  // 6V reverse
            break;
    }
}

void Intake::set_state(State state) {
    this->state = state;
}

void Intake::set_state_and_move(State state) {
    this->state = state;
    move();
}

void Intake::opcontrol(pros::Controller& controller) {
    if (controller.get_digital(DIGITAL_R1)) {
        set_state(State::INTAKING);
    } else if (controller.get_digital(DIGITAL_R2)) {
        set_state(State::OUTTAKING);
    } else if (controller.get_digital(DIGITAL_RIGHT)) {
        set_state(State::MIDDLE);
    } else {
        set_state(State::NONE);
    }

    move();
}
