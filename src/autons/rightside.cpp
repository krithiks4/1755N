#include "rightside.hpp"

// Autonomous Constants
// These are out of 127
const int DRIVE_SPEED = 127;
const int TURN_SPEED = 100;
const int SWING_SPEED = 90;

// left side speial
void right_side_auton() {
  chassis.odom_xyt_set(0_in, 0_in, -90_deg);

  // go to between matchloader and high goal
  chassis.pid_odom_set({{-2_tile + 5_in, 0_tile}, FORWARD, (int) (DRIVE_SPEED * 0.8)});
  chassis.pid_wait();

  // 2. turn towards matchloader
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait_quick();

  tongue_piston.set(true);
  pros::delay(300);

  scoring.set_state_and_move(Scoring::State::INTAKING);

  // thrusting part

  // 11. go forward 13 inches
  chassis.pid_drive_set(13_in, 0.65*DRIVE_SPEED);
  pros::delay(500);
  chassis.pid_wait();

  // 12. back 4 inches
  chassis.pid_drive_set(-4_in, 0.75*DRIVE_SPEED);
  chassis.pid_wait();

  // 13. forward 5 inches
  chassis.pid_drive_set(5_in, 0.6*DRIVE_SPEED);
  pros::delay(250);
  chassis.pid_wait();

  // 13. back away
  chassis.pid_odom_set(-5_in, DRIVE_SPEED);
  chassis.pid_wait();

  // go to low goal part

  // go to just before the balls on ground
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_odom_set({{-0.35_tile, 0.865_tile, 0_deg}, FORWARD, (int) (DRIVE_SPEED * 0.8)});
  chassis.pid_wait();

  tongue_piston.set(true);
  chassis.pid_drive_set(0.5_tile, DRIVE_SPEED*0.7);
  chassis.pid_wait();

  chassis.pid_turn_set(56_deg, TURN_SPEED);
  chassis.pid_wait();

  tongue_piston.set(false);

  chassis.pid_drive_set(0.7_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 6. low goal
  scoring.set_state_and_move(Scoring::State::OUTTAKING);
}
