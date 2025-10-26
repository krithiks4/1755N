#include "main.h"
#include "sprockets.hpp"

void Sprockets::move() {
    switch (state) {
        case State::LOW_GOAL:
            intake_motor.move(SPROCKET_VOLTAGE);
            low_motor.move(-SPROCKET_VOLTAGE);
            middle_motor.move(SPROCKET_VOLTAGE);
            high_motor.brake();
            break;
        case State::MIDDLE_GOAL:
            intake_motor.move(-SPROCKET_VOLTAGE);
            low_motor.move(SPROCKET_VOLTAGE);
            middle_motor.move(SPROCKET_VOLTAGE);
            high_motor.move(-SPROCKET_VOLTAGE);
            break;
        case State::HIGH_GOAL:
            intake_motor.move(-SPROCKET_VOLTAGE);
            low_motor.move(SPROCKET_VOLTAGE);
            middle_motor.move(-SPROCKET_VOLTAGE);
            high_motor.move(SPROCKET_VOLTAGE);
            break;
        case State::NONE:
        default:
            low_motor.brake();
            middle_motor.brake();
            high_motor.brake();
            break;
    }

    if (intake_running) {
        intake_motor.move(-SPROCKET_VOLTAGE);
    } else {
        if (state == State::NONE) intake_motor.brake();
    }
}

void Sprockets::set_state(State state) {
    this->state = state;
}

void Sprockets::set_state_and_move(State state) {
    this->state = state;
    move();
}

void Sprockets::set_intake(bool running) {
    this->intake_running = running;
}

void Sprockets::set_intake_and_move(bool running) {
    this->intake_running = running;
    move();
}

void Sprockets::opcontrol() {
    if (master.get_digital(DIGITAL_R1)) {
        set_intake(true);
    } else {
        set_intake(false);
    }

    if (master.get_digital(DIGITAL_L1)) {
        set_state(State::HIGH_GOAL);
    } else if (master.get_digital(DIGITAL_L2)) {
        set_state(State::MIDDLE_GOAL);
    } else if (master.get_digital(DIGITAL_R2)) {
        set_state(State::LOW_GOAL);
    } else {
        set_state(State::NONE);
    }

    move();
}
