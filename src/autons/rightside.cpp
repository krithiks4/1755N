#include "rightside.hpp"

// Autonomous Constants
// These are out of 127
const int DRIVE_SPEED = 105;
const int TURN_SPEED = 90;
const int SWING_SPEED = 90;

// long goal right side + middle low
void right_side_auton() {
// 1. go forward 1 tile
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_drive_set(0.865_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 2. turn towards left side
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  chassis.pid_wait();

  // 3. go forward tile and intake
  scoring.set_state_and_move(Scoring::State::INTAKING);
  chassis.pid_drive_set(0.35_tile, DRIVE_SPEED*0.4);
  chassis.pid_wait();

  scoring.set_state_and_move(Scoring::State::INTAKING);
  tongue_piston.set(true);
  chassis.pid_drive_set(0.5_tile, DRIVE_SPEED*0.7);
  chassis.pid_wait();

  // 4. turn 45 degrees counterclockwise
  chassis.pid_turn_set(124_deg, TURN_SPEED);
  chassis.pid_wait();

  // 5. move backwards 0.7 tiles
  chassis.pid_drive_set(-0.8_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 6. set state outtake for 1.25 seconds then turn off
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(1700);
  scoring.set_state_and_move(Scoring::State::INTAKING);
  tongue_piston.set(false);

  // 7. go forward 1.625 tiles to matchloader
  chassis.pid_wait();
  chassis.pid_odom_set({{1.66_tile - 2_in, -0.25_tile}, FORWARD, (int)(0.8*DRIVE_SPEED)});
  chassis.pid_wait();
  
  tongue_piston.set(true);
  chassis.pid_turn_set(180_deg, (int) (TURN_SPEED * 0.8));
  chassis.pid_wait();

  chassis.odom_xy_set(0_in, 0_in);

  // 11. go forward 13 inches
  chassis.pid_drive_set(13_in, 0.65*DRIVE_SPEED);
  pros::delay(500);
  chassis.pid_wait();

  // 12. back 4 inches
  chassis.pid_drive_set(-4_in, 0.75*DRIVE_SPEED);
  chassis.pid_wait();

  // 13. forward 5 inches
  chassis.pid_drive_set(65_in, 0.6*DRIVE_SPEED);
  pros::delay(250);
  chassis.pid_wait();

  // 14. back
  chassis.pid_odom_set({{0.5_in, 1.3_tile}, REVERSE, DRIVE_SPEED});
  chassis.pid_wait();

  // 15. set state high goal for 2 seconds
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(800);
  scoring.set_state_and_move(Scoring::State::OUTTAKING);
  pros::delay(100);
  scoring.set_state_and_move(Scoring::State::HIGH_GOAL);
  pros::delay(800);
  scoring.set_state_and_move(Scoring::State::NONE);
  chassis.odom_xyt_set(1_in, 0_in, 180_deg);

  // 16. move forward 0.75 tile
  chassis.pid_drive_set(0.75_tile, DRIVE_SPEED);
  chassis.pid_wait();

  // 17. turn towards 135 degrees then backwards 1 tile
  chassis.pid_turn_set(135_deg, TURN_SPEED);
  chassis.pid_wait();
  chassis.pid_drive_set(-0.65_tile, 0.75*DRIVE_SPEED);
  chassis.pid_wait();

  // 18. activate wing piston
  wing_piston.set(true);

  // go staight
  chassis.pid_turn_set(180_deg, TURN_SPEED);
  chassis.pid_wait();

  // 18. let wing piston fal
  wing_piston.set(false);

  // 19. move backward one tile
  chassis.pid_drive_set(-0.8_tile, 127);
  chassis.pid_wait();
}