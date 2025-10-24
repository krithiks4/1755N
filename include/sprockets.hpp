#pragma once

#include "main.h"

class Sprockets {
    public:
        enum class State {
            NONE,
            LOW_GOAL,
            MIDDLE_GOAL,
            HIGH_GOAL
        };
    private:
        bool intake_running = false;
        State state = State::NONE;
        
        pros::Motor intake_motor;
        pros::Motor indexer_motor;
        pros::Motor high_goal_motor;

        const int SPROCKET_VOLTAGE = 127;

        void move();
    public:
        Sprockets(int intake_port, int indexer_port, int high_goal_port)
            : intake_motor(intake_port), indexer_motor(indexer_port), high_goal_motor(high_goal_port) {};

        void set_state(State state);
        void set_state_and_move(State state);

        void set_intake(bool running);
        void set_intake_and_move(bool running);

        void opcontrol();
};