#include "leftside.hpp"

// long goal left side + middle high
void left_side_auton() {
  chassis.odom_xyt_set(-5_in, -1_in, 0_deg); // ORIGIN POSITION: X: EXACTLY MIDDLE OF FIELD, Y: 1 TILE from SIDE

  scoring.set_state_and_move(Scoring::State::INTAKING);

  // go to center balls
  chassis.pid_odom_set({{-1_tile, 1_tile, 0_deg}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // go to center goal
  chassis.pid_odom_set({{-0.5_tile, 1.5_tile, 45_deg}, REVERSE, DRIVE_SPEED});
  chassis.pid_wait();

  // low goal
  scoring.set_state_and_move(Scoring::State::OUTTAKING);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader (activate tonge)
  chassis.pid_odom_set({{-2_tile, 0_tile}, FORWARD, DRIVE_SPEED});
  tongue_piston.set(true);
  chassis.pid_wait();

  // 2. turn towards matchloader
  chassis.pid_turn_set({-2_tile, -1_tile}, FORWARD, TURN_SPEED);
  chassis.pid_wait_quick();

  // 4. thrust forward (without odom)
  chassis.pid_drive_set(14.9_in, DRIVE_SPEED);
  chassis.pid_wait_quick();

  // 5. intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  pros::delay(450);

  // 6. move to long goal (back some offset)
  chassis.pid_odom_set({{-2_tile, 1_tile - 10_in}, REVERSE, DRIVE_SPEED});
  tongue_piston.set(false);
  chassis.pid_wait_quick();

  // 8. run high goal
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(2000);
  scoring.set_state_and_move(Scoring::State::NONE);

  // go to between high goal and loader
  chassis.pid_odom_set({{-2_tile, 0_tile}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // ACTIVATE WING

  // go to right side of goal so that wing can be used
  chassis.pid_odom_set({{-1.5_tile, 1.4_tile, 0_deg}, FORWARD, DRIVE_SPEED});
  chassis.pid_wait();

  // go backwards to stop touching the balls????
}
