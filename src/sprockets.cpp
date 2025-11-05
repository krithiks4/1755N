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
        case State::INTAKE:
            // TODO: Use pros::Motor.torque() to stop each motor when they fill up
            intake_motor.move(-SPROCKET_VOLTAGE);
            if (low_motor.get_torque() <= 0.3) low_motor.move(SPROCKET_VOLTAGE); else low_motor.move(SPROCKET_VOLTAGE / 3);
            if (middle_motor.get_torque() <= 0.3) middle_motor.move(-SPROCKET_VOLTAGE / 3); else middle_motor.move(-SPROCKET_VOLTAGE / 4);
            high_motor.brake();
            break;
        case State::NONE:
        default:
            intake_motor.brake();
            low_motor.brake();
            middle_motor.brake();
            high_motor.brake();
            break;
    }
}

void Sprockets::set_state(State state) {
    this->state = state;
}

void Sprockets::set_state_and_move(State state) {
    this->state = state;
    move();
}

void Sprockets::opcontrol() {
    if (master.get_digital(DIGITAL_L1)) {
        set_state(State::HIGH_GOAL);
    } else if (master.get_digital(DIGITAL_L2)) {
        set_state(State::MIDDLE_GOAL);
    } else if (master.get_digital(DIGITAL_R1)) {
        set_state(State::INTAKE);
    } else if (master.get_digital(DIGITAL_R2)) {
        set_state(State::LOW_GOAL);
    } else {
        set_state(State::NONE);
    }

    move();
}
