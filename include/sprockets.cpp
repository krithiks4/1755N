#include "sprockets.hpp"

void Sprockets::move() {
    switch (state) {
        case State::LOW_GOAL:
            indexer_motor.move_voltage(SPROCKET_VOLTAGE);
            high_goal_motor.move_voltage(0);
            break;
        case State::MIDDLE_GOAL:
            indexer_motor.move_voltage(SPROCKET_VOLTAGE);
            high_goal_motor.move_voltage(SPROCKET_VOLTAGE);
            break;
        case State::HIGH_GOAL:
            indexer_motor.move_voltage(SPROCKET_VOLTAGE);
            high_goal_motor.move_voltage(-SPROCKET_VOLTAGE);
            break;
        case State::NONE:
        default:
            indexer_motor.brake();
            high_goal_motor.brake();
            break;
    }
    
    if (intake_running) {
        intake_motor.move_voltage(SPROCKET_VOLTAGE);
    } else {
        intake_motor.brake();
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
    if (master.get_digital(DIGITAL_A)) {
        set_intake(true);
    } else {
        set_intake(false);
    }

    if (master.get_digital_new_press(DIGITAL_UP)) {
        set_state(State::HIGH_GOAL);
    } else if (master.get_digital_new_press(DIGITAL_RIGHT)) {
        set_state(State::MIDDLE_GOAL);
    } else if (master.get_digital_new_press(DIGITAL_DOWN)) {
        set_state(State::LOW_GOAL);
    } else {
        set_state(State::NONE);
    }

    move();
}
