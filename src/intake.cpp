#include "main.h"
#include "intake.hpp"

void Intake::move() {
    switch (state) {
        case State::INTAKING:
            intake_motors.move(INTAKE_VOLTAGE);
            break;
        case State::OUTTAKING:
            intake_motors.move(-INTAKE_VOLTAGE);
            break;
        case State::NONE:
        default:
            intake_motors.brake();
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

void Intake::opcontrol() {
    if (master.get_digital(DIGITAL_R1)) {
        set_state(State::INTAKING);
    } else if (master.get_digital(DIGITAL_R2)) {
        set_state(State::OUTTAKING);
    } else {
        set_state(State::NONE);
    }

    move();
}
